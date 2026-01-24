import { initUI, updateState, updateSource } from "./ui.js";

// --------------------------------------------------
// WEBSOCKET
// --------------------------------------------------
const ws = new WebSocket(
  (location.protocol === "https:" ? "wss://" : "ws://") +
  location.host + "/ws"
);

ws.onopen = () => {
  console.log("[WS] connected");
  initUI();
};

// --------------------------------------------------
// GLOBAL API
// --------------------------------------------------
window.sendCmd = function (target, name, value) {
  ws.send(JSON.stringify({
    type: "cmd",
    target,
    data: buildPayload(target, name, value),
  }));
};

window.sendForce = function (web, auto) {
  ws.send(JSON.stringify({
    type: "force",
    web,
    auto,
  }));
};

window.sendEmergency = function (value) {
  ws.send(JSON.stringify({
    type: "emergency",
    value,
  }));
};

// --------------------------------------------------
// PAYLOAD BUILDER (INDEPENDENT BITS)
// --------------------------------------------------
function buildPayload(target, name, value) {
  const maps = {
    events: ["ENGINE", "CLUTCH", "SPEED", "MOVE"],
    lights: ["FP", "FS", "FL", "BACK", "LB", "RB"],
  };

  return maps[target]?.map(k => (k === name ? value : 0)) ?? [];
}

// --------------------------------------------------
// INCOMING
// --------------------------------------------------
ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);

  if (msg.type === "state") {
    updateState(msg.name, msg.value);
  }

  if (msg.type === "source") {
    updateSource(msg.value);
  }
};