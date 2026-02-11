//tomo_web/html/ws.js

import {
  initUI,
  updateState,
  updateSource,
  updateTelemetry
} from "./ui.js";
import { SOURCE, CATEGORY, SYSTEM } from "./ros_enums.js";

const ws = new WebSocket(
  (location.protocol === "https:" ? "wss://" : "ws://") +
  location.host + "/ws"
);

ws.onopen = () => {
  console.log("[WS] connected");
  initUI();
};


// ==================================================
// SEND NORMAL EVENT / STATE
// ==================================================
window.sendEvent = function (meta, name) {
  ws.send(JSON.stringify({
    type: "event",
    payload: {
      source: SOURCE.WEB,
      category: meta.category, // dolazi iz STATE_MAP
      type: meta.code,         // dolazi iz STATE_MAP
      value: 1
    }
  }));
};

// ==================================================
// EMERGENCY (HARD / SOFT / RELEASE)
// ==================================================
window.sendEmergency = function (payload) {
  ws.send(JSON.stringify({
    type: "emergency",
    active: payload.active,
    level: payload.level
  }));
};

// ==================================================
// FORCE SOURCE
// ==================================================
window.sendSource = function (label) {

  let value = SOURCE.PS4;

  if (label.includes("WEB"))  value = SOURCE.WEB;
  if (label.includes("AUTO")) value = SOURCE.AUTO;

  ws.send(JSON.stringify({
    type: "event",
    payload: {
      source: SOURCE.WEB,
      category: CATEGORY.SYSTEM,
      type: SYSTEM.FORCE_SOURCE,
      value: value
    }
  }));
};

// ==================================================
// RECEIVE OUTPUT FROM FACTORY
// ==================================================
ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);

  if (msg.type === "output") {

    const d = msg.data;

    // ---- SOURCE always from ROS----
    updateSource(d.source);

    // ---- STATES ----
    updateState("ARMED", d.armed ? "1" : "0");
    updateState("POWER", d.power ? "1" : "0");
    updateState("LIGHT", d.light ? "1" : "0");

    // ---- EVENTS ----
    updateState("ENGINE_START", d.engine_start ? "1" : "0");
    updateState("ENGINE_STOP", d.engine_stop ? "1" : "0");
    updateState("CLUTCH", d.clutch ? "1" : "0");
    updateState("BRAKE", d.brake ? "1" : "0");
    updateState("MOVE", d.move ? "1" : "0");

    // ---- LIGHTS ----
    updateState("FP", d.fp ? "1" : "0");
    updateState("FS", d.fs ? "1" : "0");
    updateState("FL", d.fl ? "1" : "0");
    updateState("BP", d.bp ? "1" : "0");
    updateState("LB", d.lb ? "1" : "0");
    updateState("RB", d.rb ? "1" : "0");
  }

  if (msg.type === "esp_state") {
    updateState(msg.name, msg.value);
  }

  if (msg.type === "ros_telemetry") {
    updateTelemetry(msg.name, msg.value);
  }
};
