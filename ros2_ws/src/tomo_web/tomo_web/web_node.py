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
from std_msgs.msg import String
from tomo_msgs.msg import ControlEvents, Emergency, OutputStates

from ament_index_python.packages import get_package_share_directory

UDP_PORT = 9999
LOG_LIMIT = 200

# ==================================================
# ROS ↔ WEB BRIDGE NODE
# ==================================================
class WebRosBridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('tomo_web')

        self.loop = loop

        # -------- PUBLISHERS --------
        self.pub_control = self.create_publisher(
            ControlEvents, 'control/events', 10
        )
        self.pub_emergency = self.create_publisher(
            Emergency, 'control/emergency', 10
        )

        # -------- SUBSCRIBERS --------
        self.create_subscription(
            OutputStates,
            'tomo/output',
            self.output_cb,
            10
        )

    # ---------- SEND CONTROL EVENT ----------
    def send_event(self, payload: dict):
        msg = ControlEvents()
        msg.source = payload['source']
        msg.category = payload['category']
        msg.type = payload['type']
        msg.value = payload.get('value', 0)
        msg.linear = payload.get('linear', 0.0)
        msg.angular = payload.get('angular', 0.0)
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_control.publish(msg)

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
                    'emergency': msg.emergency,
                    'armed': msg.armed,
                    'power': msg.power_mode,
                    'light': msg.light_mode,
                    'engine': msg.engine_start,
                    'clutch': msg.clutch_down,
                    'speed': msg.high_speed,
                    'move': msg.move_allowed,
                    'fp': msg.front_position,
                    'fs': msg.front_short,
                    'fl': msg.front_long,
                    'back': msg.back_light,
                    'lb': msg.left_blink,
                    'rb': msg.right_blink,
                    'source': msg.active_source,
                }
            }),
            self.loop
        )

# ==================================================
# FASTAPI
# ==================================================
app = FastAPI()
ros_node: WebRosBridge | None = None

html_dir = files("tomo_web") / "html"

app.mount("/", StaticFiles(directory=html_dir, html=True), name="html")

WS_CLIENTS: set[WebSocket] = set()

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    WS_CLIENTS.add(ws)
    try:
        while True:
            data = await ws.receive_json()

            if data['type'] == 'event' and ros_node:
                ros_node.send_event(data['payload'])

            if data['type'] == 'emergency' and ros_node:
                ros_node.send_emergency(
                    data['active'],
                    data.get('level', 1),
                    'web'
                )

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

# ==================================================
# UDP LISTENER (ESP → WEB LOGS)
# ==================================================
def udp_listener(loop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))

    while True:
        data, _ = sock.recvfrom(512)
        msg = data.decode(errors="ignore").strip()
        asyncio.run_coroutine_threadsafe(
            broadcast({'type': 'log', 'text': msg}),
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
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info')
