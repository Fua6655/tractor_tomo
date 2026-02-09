#!/usr/bin/env python3
import asyncio
import socket
import threading
from importlib.resources import files

from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse
from starlette.websockets import WebSocketDisconnect

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tomo_msgs.msg import ControlEvents, Emergency, OutputStates

UDP_PORT = 9999
ESP_UDP_PORT = 8888
LOG_LIMIT = 200


# ==================================================
# ROS ↔ WEB BRIDGE NODE
# ==================================================
class WebRosBridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('tomo_web')

        self.declare_parameter('control_event_topic', '/control/events')
        self.declare_parameter('control_emergency_topic', '/control/emergency')
        self.declare_parameter("output_topic", "/tomo/states")
        self.declare_parameter("engine_cmd_topic", "/tomo/engine_cmd")
        self.declare_parameter("steer_cmd_topic", "/tomo/steer_cmd")

        self.control_event_topic = str(self.get_parameter('control_event_topic').value)
        self.control_emergency_topic = str(self.get_parameter('control_emergency_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.engine_cmd_topic = str(self.get_parameter('engine_cmd_topic').value)
        self.steer_cmd_topic = str(self.get_parameter('steer_cmd_topic').value)

        self.loop = loop
        self.feedback_source = "ROS"

        # -------- PUBLISHERS --------
        self.pub_event = self.create_publisher(ControlEvents, self.control_event_topic, 50)
        self.pub_emergency = self.create_publisher(Emergency, self.control_emergency_topic, 10)

        # -------- SUBSCRIBERS --------
        self.create_subscription(OutputStates, self.output_topic, self.output_cb, 20)
        self.create_subscription(Float32, self.engine_cmd_topic, self.engine_cmd_cb, 20)
        self.create_subscription(Float32, self.steer_cmd_topic, self.steer_cmd_cb, 20)

    # ---------- SEND CONTROL EVENT ----------
    def send_event(self, payload: dict):
        msg = ControlEvents()
        msg.source = payload['source']
        msg.category = payload['category']
        msg.type = payload['type']
        msg.value = payload.get('value', 0)
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_event.publish(msg)

    # ---------- EMERGENCY ----------
    def send_emergency(self, active: bool, level: int, reason: str):
        msg = Emergency()
        msg.active = active
        msg.level = level
        msg.reason = reason
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_emergency.publish(msg)

    # ---------- OUTPUT FEEDBACK ----------
    def output_cb(self, msg: OutputStates):
        asyncio.run_coroutine_threadsafe(
            broadcast({
                'type': 'output',
                'data': {
                    'source': msg.source,
                    'armed': msg.armed_state,
                    'power': msg.power_state,
                    'light': msg.light_state,
                    'engine_start': msg.engine_start,
                    'engine_stop': msg.engine_stop,
                    'clutch': msg.clutch_active,
                    'brake': msg.brake_active,
                    'move': msg.move_allowed,
                    'fp': msg.front_position,
                    'fs': msg.front_short,
                    'fl': msg.front_long,
                    'bp': msg.back_position,
                    'lb': msg.left_blink,
                    'rb': msg.right_blink,
                }
            }),
            self.loop
        )

    def engine_cmd_cb(self, msg: Float32):
        if self.feedback_source != "ROS":
            return

        asyncio.run_coroutine_threadsafe(
            broadcast({
                "type": "ros_telemetry",
                "name": "THR",
                "value": msg.data,
            }),
            self.loop
        )

    def steer_cmd_cb(self, msg: Float32):
        if self.feedback_source != "ROS":
            return

        asyncio.run_coroutine_threadsafe(
            broadcast({
                "type": "ros_telemetry",
                "name": "STR",
                "value": msg.data,
            }),
            self.loop
        )


# ==================================================
# FASTAPI
# ==================================================
ros_node: WebRosBridge | None = None
app = FastAPI()

html_dir = files("tomo_web") / "html"
app.mount("/html", StaticFiles(directory=str(html_dir)), name="html")


@app.get("/")
def root():
    return RedirectResponse("/html/index.html")


WS_CLIENTS: set[WebSocket] = set()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    client_ip = ws.client.host
    WS_CLIENTS.add(ws)

    try:
        while True:
            data = await ws.receive_json()

            if data['type'] == 'feedback_source':
                ros_node.feedback_source = data['value']
                continue

            if data['type'] == 'event':
                ros_node.send_event(data['payload'])

            if data['type'] == 'emergency':
                ros_node.send_emergency(
                    data['active'],
                    data.get('level', 1),
                    'web'
                )

            if data['type'] == 'heartbeat':
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(b"HEARTBEAT", (client_ip, ESP_UDP_PORT))
                sock.close()

    except WebSocketDisconnect:
        pass
    finally:
        WS_CLIENTS.discard(ws)


async def broadcast(payload: dict):
    for ws in list(WS_CLIENTS):
        try:
            await ws.send_json(payload)
        except:
            WS_CLIENTS.discard(ws)


def udp_listener(loop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))

    while True:
        data, _ = sock.recvfrom(512)
        msg = data.decode(errors="ignore").strip()

        if msg.startswith("STATE,"):
            try:
                _, name, value = msg.split(",", 2)
                asyncio.run_coroutine_threadsafe(
                    broadcast({
                        "type": "esp_state",
                        "name": name,
                        "value": value,
                    }),
                    loop
                )
                continue
            except:
                pass

        asyncio.run_coroutine_threadsafe(
            broadcast({'type': 'log', 'text': msg}),
            loop
        )


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
    uvicorn.run("tomo_web.web_node:app", host='0.0.0.0', port=8000, log_level='info')
