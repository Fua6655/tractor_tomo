#!/usr/bin/env python3

import asyncio
import threading
from fastapi import FastAPI, WebSocket
from starlette.websockets import WebSocketDisconnect
from importlib.resources import files
from fastapi.staticfiles import StaticFiles
import rclpy

from .web_node import WebNode

app = FastAPI()
WS_CLIENTS = set()
ros_node = None

html_dir = files("tomo_web") / "web"
app.mount("/", StaticFiles(directory=str(html_dir), html=True), name="web")

# ==================================================
@app.websocket("/ws")
async def ws(ws: WebSocket):
    await ws.accept()
    WS_CLIENTS.add(ws)

    try:
        while True:
            msg = await ws.receive_json()

            if msg["type"] == "event":
                ros_node.emit_event(msg["payload"])

            elif msg["type"] == "emergency":
                ros_node.emit_emergency(msg["value"])

    except WebSocketDisconnect:
        WS_CLIENTS.discard(ws)


# ==================================================
async def broadcast(payload):
    for ws in list(WS_CLIENTS):
        try:
            await ws.send_json(payload)
        except:
            WS_CLIENTS.discard(ws)


# ==================================================
def main():
    global ros_node
    loop = asyncio.get_event_loop()

    rclpy.init()
    ros_node = WebNode(loop)

    threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True
    ).start()

    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
