// tomo_web/html/ui.js

import { STATE_MAP } from "./state.js";
import { SOURCE } from "./ros_enums.js";

let containers = {};
let pageWeb = null;
let pageAuto = null;

// 🔥 renamed from espTelemetry
let telemetry = {
  THR: 0.0,
  STR: 0.0,
};

const buttons = {};
let activeSource = "PS4";
let feedbackSource = "ROS";
let failsafeActive = false;

const SOURCE_TO_BUTTON = {
  PS4: "PS4_CTRL",
  WEB: "WEB_CTRL",
  AUTO: "AUTO_CTRL",
};

// --------------------------------------------------
export function initUI() {

  containers = {
    source: document.getElementById("source"),
    feedback: document.getElementById("feedback"),
    safety: document.getElementById("safety"),
    states: document.getElementById("states"),
    events: document.getElementById("events"),
    lights: document.getElementById("lights"),
  };

  pageWeb  = document.getElementById("page-web");
  pageAuto = document.getElementById("page-auto");

  Object.entries(STATE_MAP).forEach(([name, meta]) => {
    getButton(name, meta);
  });
}

// --------------------------------------------------
function getButton(name, meta) {
  if (buttons[name]) return buttons[name];

  const btn = document.createElement("div");
  btn.className = "btn off";
  btn.textContent = meta.label;

  if (meta.type === "emergency_hard") {
    btn.style.background = "#c62828";
    btn.style.color = "white";
  }
  if (meta.type === "emergency_soft") {
    btn.style.background = "#ef6c00";
    btn.style.color = "black";
  }
  if (meta.type === "emergency_release") {
    btn.style.background = "#2e7d32";
    btn.style.color = "white";
  }

  btn.onclick = () => {

    if (meta.type === "failsafe") return;

    if (meta.type === "emergency_hard") {
      window.sendEmergency({ active: true, level: 1 });
      return;
    }

    if (meta.type === "emergency_soft") {
      window.sendEmergency({ active: true, level: 0 });
      return;
    }

    if (meta.type === "emergency_release") {
      window.sendEmergency({ active: false });
      return;
    }

    if (meta.type === "source") {
      window.sendSource(meta.label.toUpperCase());
      return;
    }

    if (meta.type === "feedback") {
      feedbackSource = meta.label;

      ["FEEDBACK_ROS", "FEEDBACK_ESP"].forEach(k => {
        if (buttons[k]) {
          buttons[k].className =
            (buttons[k].textContent === feedbackSource) ? "btn on" : "btn off";
        }
      });

      window.sendFeedbackSource(feedbackSource);
      updateTelemetryUI();   // 🔥 refresh immediately
      return;
    }

    window.sendEvent(meta, name);
  };

  if (!containers[meta.group]) return;

  containers[meta.group].appendChild(btn);
  buttons[name] = btn;
  return btn;
}

// --------------------------------------------------
export function updateSource(source) {

  if (source === SOURCE.WEB) activeSource = "WEB";
  else if (source === SOURCE.AUTO) activeSource = "AUTO";
  else activeSource = "PS4";

  pageWeb.style.display  = activeSource === "AUTO" ? "none"  : "block";
  pageAuto.style.display = activeSource === "AUTO" ? "block" : "none";

  ["PS4_CTRL", "WEB_CTRL", "AUTO_CTRL"].forEach((k) => {
    if (buttons[k]) {
      buttons[k].className =
        (activeSource === k.replace("_CTRL", "")) ? "btn on" : "btn off";
    }
  });

  updateControlLock();
}

// --------------------------------------------------
export function updateState(name, value) {

  const btn = buttons[name];
  if (!btn) return;

  const meta = STATE_MAP[name];
  if (!meta) return;

  if (meta.group === "source") return;

  if (name === "FAILSAFE") {
    failsafeActive = value === "1";
    btn.classList.remove("on", "off");
    btn.classList.add(failsafeActive ? "off" : "on");
    updateControlLock();
    return;
  }

  btn.className = value === "1" ? "btn on" : "btn off";
}

// --------------------------------------------------
export function updateTelemetry(name, value) {
  telemetry[name] = parseFloat(value);
  updateTelemetryUI();
}

// --------------------------------------------------
export function getFeedbackSource() {
  return feedbackSource;
}

// --------------------------------------------------
function updateControlLock() {
  Object.entries(buttons).forEach(([name, btn]) => {
    const meta = STATE_MAP[name];
    if (!meta) return;

    if (
      meta.group === "source" ||
      meta.group === "safety" ||
      meta.group === "feedback"
    ) {
      btn.classList.remove("disabled");
      return;
    }

    if (failsafeActive) {
      btn.classList.add("disabled");
      return;
    }
  });
}

// --------------------------------------------------
function updateTelemetryUI() {

  const thrEl = document.getElementById("thr-value");
  const strEl = document.getElementById("str-value");
  if (!thrEl || !strEl) return;

  if (feedbackSource === "ESP") {
    thrEl.textContent = telemetry.THR.toFixed(3);
    strEl.textContent = telemetry.STR.toFixed(3);
  } else {
    thrEl.textContent = telemetry.THR.toFixed(6);
    strEl.textContent = telemetry.STR.toFixed(6);
  }
}
