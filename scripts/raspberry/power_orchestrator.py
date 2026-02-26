#!/usr/bin/env python3
import json
import os
import re
import shlex
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Optional


class PowerOrchestrator:
    def __init__(self):
        self.repo_dir = Path(os.environ.get("TOMO_REPO_DIR", str(Path.home() / "tractor_tomo")))
        self.power_manager_host = os.environ.get("POWER_MANAGER_HOST", "192.168.0.185")
        self.power_manager_port = int(os.environ.get("POWER_MANAGER_PORT", "5005"))
        self.local_udp_port = int(os.environ.get("POWER_MANAGER_LOCAL_PORT", "5006"))

        self.loop_hz = float(os.environ.get("ORCH_LOOP_HZ", "2.0"))
        self.heartbeat_period = float(os.environ.get("ORCH_HEARTBEAT_SEC", "1.0"))
        self.ros_warmup_sec = float(os.environ.get("ORCH_ROS_WARMUP_SEC", "8.0"))
        self.agent_check_sec = float(os.environ.get("ORCH_AGENT_CHECK_SEC", "1.5"))
        self.shutdown_on_remote = os.environ.get("ORCH_EXEC_SHUTDOWN", "1") == "1"

        self.ros_proc: Optional[subprocess.Popen] = None
        self.agent_proc: Optional[subprocess.Popen] = None
        self.running = True
        self.shutdown_requested = False
        self.ros_ready_sent = False
        self.agent_ready_sent = False
        self.last_heartbeat_sent = 0.0
        self.last_agent_check = 0.0
        self.last_agent_alive = False
        self.ack_received = set()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.local_udp_port))
        self.sock.setblocking(False)

        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _handle_signal(self, signum, _frame):
        self.log(f"signal={signum}, stopping orchestrator")
        self.running = False

    def log(self, msg: str):
        print(f"[power-orchestrator] {msg}", flush=True)

    def send_text(self, payload: str):
        self.sock.sendto(payload.encode("utf-8"), (self.power_manager_host, self.power_manager_port))

    def send_reliable(self, payload: str, repeats: int = 5, delay_s: float = 0.15):
        for _ in range(max(1, repeats)):
            try:
                self.send_text(payload)
            except Exception:
                pass
            time.sleep(delay_s)

    def send_heartbeat(self):
        load_1m = self.get_load_1m()
        temp_c = self.get_temp_c()
        msg = {
            "ros": self.is_proc_alive(self.ros_proc),
            "agent": self.last_agent_alive and self.is_proc_alive(self.agent_proc),
            "load": round(load_1m, 2),
            "temp": round(temp_c, 2),
        }
        self.send_text("HEARTBEAT")
        self.send_text(json.dumps(msg))

    def get_load_1m(self) -> float:
        try:
            return float(os.getloadavg()[0])
        except Exception:
            return 0.0

    def get_temp_c(self) -> float:
        candidates = [
            "/sys/class/thermal/thermal_zone0/temp",
            "/sys/class/hwmon/hwmon0/temp1_input",
        ]
        for path in candidates:
            try:
                raw = Path(path).read_text(encoding="utf-8").strip()
                val = float(raw)
                if val > 1000.0:
                    return val / 1000.0
                return val
            except Exception:
                continue
        return 0.0

    def is_proc_alive(self, proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def spawn_script(self, script_rel_path: str) -> subprocess.Popen:
        script_abs = self.repo_dir / script_rel_path
        cmd = f"cd {shlex.quote(str(self.repo_dir))} && {shlex.quote(str(script_abs))}"
        self.log(f"starting {script_rel_path}")
        return subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid,
        )

    def stop_proc_group(self, proc: Optional[subprocess.Popen], name: str, timeout: float = 8.0):
        if proc is None or proc.poll() is not None:
            return
        self.log(f"stopping {name}")
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            started = time.monotonic()
            while (time.monotonic() - started) < timeout:
                if proc.poll() is not None:
                    return
                time.sleep(0.2)
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def topic_has_publisher(self, topic_name: str) -> bool:
        cmd = ["ros2", "topic", "info", topic_name]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=2.5, check=False)
        except Exception:
            return False

        text = (res.stdout or "") + "\n" + (res.stderr or "")
        m = re.search(r"Publisher count:\s*(\d+)", text)
        return bool(m and int(m.group(1)) > 0)

    def check_micro_ros_esp_alive(self) -> bool:
        # Both micro-ROS ESP boards must be present.
        esp1_alive = self.topic_has_publisher("/tomo/esp1_alive")
        esp2_alive = self.topic_has_publisher("/tomo/esp2_alive")
        return esp1_alive and esp2_alive

    def handle_udp_messages(self):
        while True:
            try:
                data, _addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                break

            payload = data.decode("utf-8", errors="ignore").strip().upper()
            if not payload:
                continue

            if payload == "PING":
                continue
            if payload == "SHUTDOWN":
                self.log("received SHUTDOWN from power manager")
                self.shutdown_requested = True
                continue
            if payload.startswith("ACK_"):
                self.ack_received.add(payload)
                self.log(f"received {payload}")
                continue

    def wait_for_ack(self, ack_name: str, timeout_s: float = 4.0) -> bool:
        end_t = time.monotonic() + timeout_s
        while time.monotonic() < end_t:
            self.handle_udp_messages()
            if ack_name in self.ack_received:
                return True
            time.sleep(0.05)
        return False

    def send_with_ack(self, msg: str, ack: str, retries: int = 8, timeout_s: float = 1.2) -> bool:
        self.ack_received.discard(ack)
        for _ in range(max(1, retries)):
            self.send_reliable(msg, repeats=1, delay_s=0.0)
            if self.wait_for_ack(ack, timeout_s=timeout_s):
                return True
        return False

    def soft_boot_sequence(self):
        # 1) Initial contact so ESP enters WAITING_ROS_READY (LED1 blink)
        self.log("sending initial heartbeat/contact to power manager")
        for _ in range(3):
            self.send_heartbeat()
            time.sleep(0.2)

        # 2) Start ROS stack (tomo_launch.sh), then notify ROS_READY
        self.ros_proc = self.spawn_script("scripts/tomo_launch.sh")
        time.sleep(self.ros_warmup_sec)

        if self.is_proc_alive(self.ros_proc):
            self.send_text("ROS_READY")
            self.ros_ready_sent = True
            self.log("ROS_READY sent")
        else:
            self.log("tomo_launch.sh exited early; keeping ROS_READY=false")

        # 3) Start micro-ROS agent script (LED2 should blink in WAITING_AGENT_READY)
        self.agent_proc = self.spawn_script("scripts/start_micro_ros_agent.sh")

    def stop_micro_ros_agent_phase(self):
        self.log("shutdown phase 1: stopping micro-ROS agent")
        self.stop_proc_group(self.agent_proc, "micro_ros_agent")
        self.agent_ready_sent = False
        self.last_agent_alive = False

        stop_script = self.repo_dir / "scripts/stop_micro_ros_agent.sh"
        if stop_script.exists():
            try:
                subprocess.run(
                    ["bash", "-lc", f"cd {shlex.quote(str(self.repo_dir))} && {shlex.quote(str(stop_script))}"],
                    timeout=10,
                    check=False,
                )
            except Exception:
                pass

        self.send_reliable("AGENT_STOPPED")
        if self.send_with_ack("AGENT_STOPPED", "ACK_AGENT_STOPPED"):
            self.log("sent AGENT_STOPPED + ACK received")
        else:
            self.log("sent AGENT_STOPPED, ACK timeout")

    def stop_ros_phase(self):
        self.log("shutdown phase 2: stopping tomo_launch")
        self.stop_proc_group(self.ros_proc, "tomo_launch")
        self.ros_ready_sent = False
        if self.send_with_ack("ROS_STOPPED", "ACK_ROS_STOPPED"):
            self.log("sent ROS_STOPPED + ACK received")
        else:
            self.log("sent ROS_STOPPED, ACK timeout")

    def final_shutdown_handshake(self):
        self.log("shutdown phase 3: sending LAST_MESSAGE and SAFE_TO_POWER_OFF")
        if self.send_with_ack("LAST_MESSAGE", "ACK_LAST_MESSAGE"):
            self.log("sent LAST_MESSAGE + ACK received")
        else:
            self.log("sent LAST_MESSAGE, ACK timeout")
        time.sleep(0.4)
        if self.send_with_ack("SAFE_TO_POWER_OFF", "ACK_SAFE_TO_POWER_OFF", retries=12):
            self.log("sent SAFE_TO_POWER_OFF + ACK received")
        else:
            self.log("sent SAFE_TO_POWER_OFF, ACK timeout")

    def trigger_local_shutdown(self):
        if not self.shutdown_on_remote:
            self.log("ORCH_EXEC_SHUTDOWN=0, skipping local shutdown command")
            return

        self.log("executing local shutdown command")
        # Requires sudoers rule for password-less shutdown.
        subprocess.Popen(["sudo", "/sbin/shutdown", "-h", "now"])

    def run(self):
        self.soft_boot_sequence()

        period = 1.0 / max(self.loop_hz, 0.5)
        while self.running and not self.shutdown_requested:
            now = time.monotonic()

            self.handle_udp_messages()

            if self.ros_proc and self.ros_proc.poll() is not None and self.ros_ready_sent:
                self.log("tomo_launch exited unexpectedly")
                self.ros_ready_sent = False

            if self.agent_proc and self.agent_proc.poll() is not None:
                self.agent_ready_sent = False
                self.last_agent_alive = False

            if now - self.last_agent_check >= self.agent_check_sec:
                self.last_agent_check = now
                if self.is_proc_alive(self.agent_proc) and self.is_proc_alive(self.ros_proc):
                    self.last_agent_alive = self.check_micro_ros_esp_alive()
                else:
                    self.last_agent_alive = False

                if self.last_agent_alive and not self.agent_ready_sent:
                    self.send_text("AGENT_READY")
                    self.agent_ready_sent = True
                    self.log("AGENT_READY sent (micro-ROS ESP alive)")
                elif not self.last_agent_alive:
                    self.agent_ready_sent = False

            if now - self.last_heartbeat_sent >= self.heartbeat_period:
                self.last_heartbeat_sent = now
                self.send_heartbeat()

            time.sleep(period)

        self.log("shutdown sequence started")
        self.stop_micro_ros_agent_phase()
        self.stop_ros_phase()
        self.final_shutdown_handshake()
        if self.shutdown_requested:
            self.trigger_local_shutdown()


def main():
    orch = PowerOrchestrator()
    orch.run()


if __name__ == "__main__":
    main()
