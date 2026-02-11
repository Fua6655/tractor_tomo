// tomo_web/html/ui.js

import { STATE_MAP } from "./state.js";
import { SOURCE } from "./ros_enums.js";
let containers = {};
let pageWeb = null;
let pageAuto = null;

// Telemetry values for THR/STR display
let telemetry = {
  THR: 0.0,
  STR: 0.0,
};

const buttons = {};
let activeSource = "PS4";
let failsafeActive = false;

// --------------------------------------------------
export function initUI() {

  containers = {
    source: document.getElementById("source"),
    safety: document.getElementById("safety"),
    states: document.getElementById("states"),
    events: document.getElementById("events"),
    lights: document.getElementById("lights"),
  };

  pageWeb  = document.getElementById("page-web");
  pageAuto = document.getElementById("page-auto");

  console.log("✅ UI init, containers:", containers);

  Object.entries(STATE_MAP).forEach(([name, meta]) => {
    getButton(name, meta);
  });

  updateTelemetryUI();
}

// --------------------------------------------------
function getButton(name, meta) {
  if (buttons[name]) return buttons[name];

  const btn = document.createElement("div");
  btn.className = "btn off";
  btn.textContent = meta.label;

  // ----- EMERGENCY STYLES -----
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

    window.sendEvent(meta, name);
  };

  if (!containers[meta.group]) {
    console.error("❌ Missing container:", meta.group);
    return;
  }

  containers[meta.group].appendChild(btn);
  buttons[name] = btn;
  return btn;
}

// --------------------------------------------------
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

  // =========================
  // FAILSAFE (APSOLUTNA ISTINA)
  // =========================
  if (name === "FAILSAFE") {
    failsafeActive = value === "1";

    // OFF -> zeleno
    // ON  -> crveno
    btn.classList.remove("on", "off");
    btn.classList.add(failsafeActive ? "off" : "on");

    updateControlLock();
    return;
  }

  btn.className = value === "1" ? "btn on" : "btn off";
}

// --------------------------------------------------
// --------------------------------------------------
export function updateTelemetry(name, value) {
  telemetry[name] = parseFloat(value);
  updateTelemetryUI();
}

// --------------------------------------------------
function updateControlLock() {
  Object.entries(buttons).forEach(([name, btn]) => {
    const meta = STATE_MAP[name];
    if (!meta) return;

    // OVI SU UVIJEK AKTIVNI
    if (
      meta.group === "source" ||
      meta.group === "safety"
    ) {
      btn.classList.remove("disabled");
      return;
    }

    // =========================
    // FAILSAFE IMA PRIORITET
    // =========================
    if (failsafeActive) {
      btn.classList.add("disabled");
      return;
    }

    // =========================
    // NORMALNA SOURCE LOGIKA
    // =========================
    //if (activeSource === "WEB") {
    //  btn.classList.remove("disabled");
    //}
    //else {
    //  btn.classList.add("disabled");
    //}
  });
}

// --------------------------------------------------
function updateTelemetryUI() {
  const thrEl = document.getElementById("thr-value");
  const strEl = document.getElementById("str-value");
  if (!thrEl || !strEl) return;

  thrEl.textContent = telemetry.THR.toFixed(3);
  strEl.textContent = telemetry.STR.toFixed(3);
}
