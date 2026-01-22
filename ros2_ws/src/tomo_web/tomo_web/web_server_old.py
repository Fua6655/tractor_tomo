#!/usr/bin/env python3
import asyncio
import socket
import threading

from fastapi import FastAPI, WebSocket
from importlib.resources import files
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse
from starlette.websockets import WebSocketDisconnect

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String, Bool

UDP_PORT = 9999
LOG_LIMIT = 200

# ==================================================
# ROS ↔ WEB BRIDGE
# ==================================================
class WebRosBridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("web_ros_bridge")

        self.loop = loop

        # -------- FORCE STATE --------
        self.force_web = False
        self.force_auto = False

        # -------- SUBSCRIBERS --------
        self.create_subscription(
            String, "factory/active_source", self.source_cb, 10
        )

        # -------- PUBLISHERS --------
        self.pub_states = self.create_publisher(UInt8MultiArray, "web/states", 10)
        self.pub_events = self.create_publisher(UInt8MultiArray, "web/events", 10)
        self.pub_lights = self.create_publisher(UInt8MultiArray, "web/lights", 10)

        self.pub_force_web = self.create_publisher(Bool, "web/force_control", 10)
        self.pub_force_auto = self.create_publisher(Bool, "auto/force_control", 10)

        self.pub_emergency = self.create_publisher(Bool, "web/emergency", 10)

    # --------------------------------------------------
    # COMMANDS FROM WEB
    # --------------------------------------------------
    def publish(self, target: str, data: list[int]):
        msg = UInt8MultiArray(data=data)
        if target == "events":
            self.pub_events.publish(msg)
        elif target == "lights":
            self.pub_lights.publish(msg)
        elif target == "states":
            self.pub_states.publish(msg)

    # --------------------------------------------------
    # FORCE SOURCE (EXCLUSIVE)
    # --------------------------------------------------
    def force_mode(self, web: bool, auto: bool):
        self.force_web = bool(web)
        self.force_auto = bool(auto)

        # ekskluzivnost
        if self.force_web:
            self.force_auto = False
        if self.force_auto:
            self.force_web = False

        self.pub_force_web.publish(Bool(data=self.force_web))
        self.pub_force_auto.publish(Bool(data=self.force_auto))

        self.get_logger().info(
            f"FORCE → WEB={self.force_web}, AUTO={self.force_auto}"
        )

    # --------------------------------------------------
    # EMERGENCY
    # --------------------------------------------------
    def emergency(self, enable: bool):
        self.pub_emergency.publish(Bool(data=enable))
        self.get_logger().warn(f"EMERGENCY = {enable}")

    # --------------------------------------------------
    # SOURCE FEEDBACK → UI
    # --------------------------------------------------
    def source_cb(self, msg: String):
        asyncio.run_coroutine_threadsafe(
            broadcast({"type": "source", "value": msg.data}),
            self.loop
        )


# ==================================================
# FASTAPI
# ==================================================
ros_node: WebRosBridge | None = None

app = FastAPI()

html_dir = files("control_tomo.web") / "html"
app.mount("/html", StaticFiles(directory=str(html_dir)), name="html")


@app.get("/")
def root():
    return RedirectResponse("/html/index.html")


WS_CLIENTS: set[WebSocket] = set()
states: dict[str, str] = {}
log_buffer: list[str] = []


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    WS_CLIENTS.add(ws)

    await ws.send_json({
        "type": "init",
        "states": states,
        "logs": log_buffer
    })

    try:
        while True:
            data = await ws.receive_json()

            # ---------- NORMAL COMMAND ----------
            if data.get("type") == "cmd":
                if ros_node:
                    ros_node.publish(
                        data.get("target"),
                        data.get("data", [])
                    )

            # ---------- FORCE SOURCE ----------
            elif data.get("type") == "force":
                if ros_node:
                    ros_node.force_mode(
                        web=bool(data.get("web", False)),
                        auto=bool(data.get("auto", False))
                    )

            # ---------- EMERGENCY ----------
            elif data.get("type") == "emergency":
                if ros_node:
                    ros_node.emergency(bool(data.get("value", False)))

    except WebSocketDisconnect:
        pass
    finally:
        WS_CLIENTS.discard(ws)


# ==================================================
# BROADCAST TO ALL WS CLIENTS
# ==================================================
async def broadcast(payload: dict):
    for ws in list(WS_CLIENTS):
        try:
            await ws.send_json(payload)
        except:
            WS_CLIENTS.discard(ws)


# ==================================================
# UDP LISTENER (ESP → WEB)
# ==================================================
def udp_listener(loop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))

    while True:
        data, _ = sock.recvfrom(512)
        msg = data.decode(errors="ignore").strip()

        if msg.startswith("STATE,"):
            _, name, value = msg.split(",", 2)
            states[name] = value
            asyncio.run_coroutine_threadsafe(
                broadcast({"type": "state", "name": name, "value": value}),
                loop
            )
        else:
            log_buffer.append(msg)
            log_buffer[:] = log_buffer[-LOG_LIMIT:]
            asyncio.run_coroutine_threadsafe(
                broadcast({"type": "log", "text": msg}),
                loop
            )


# ==================================================
# STARTUP
# ==================================================
@app.on_event("startup")
async def startup():
    global ros_node
    loop = asyncio.get_running_loop()

    rclpy.init()
    ros_node = WebRosBridge(loop)

    threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True
    ).start()

    threading.Thread(
        target=udp_listener, args=(loop,), daemon=True
    ).start()


def main():
    import uvicorn
    uvicorn.run(
        "control_tomo.web.web_server:app",
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )