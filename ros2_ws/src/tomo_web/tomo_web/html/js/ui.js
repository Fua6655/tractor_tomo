import { STATE_MAP } from "./state.js";

const containers = {
  source: document.getElementById("source"),
  safety: document.getElementById("safety"),
  states: document.getElementById("states"),
  events: document.getElementById("events"),
  lights: document.getElementById("lights"),
};

const pageWeb  = document.getElementById("page-web");
const pageAuto = document.getElementById("page-auto");

const buttons = {};
let activeSource = "PS4";   // samo UI refleksija

// --------------------------------------------------
export function getButton(name, meta) {
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

    // ---------- FAILSAFE ----------
    if (meta.type === "failsafe") return;

    // ---------- HARD EMERGENCY ----------
    if (meta.type === "emergency_hard") {
      window.sendEmergency({ active: true, level: 1 });
      return;
    }

    // ---------- SOFT EMERGENCY ----------
    if (meta.type === "emergency_soft") {
      window.sendEmergency({ active: true, level: 0 });
      return;
    }

    // ---------- RELEASE EMERGENCY ----------
    if (meta.type === "emergency_release") {
      window.sendEmergency({ active: false });
      return;
    }

    // ---------- SOURCE ----------
    if (meta.type === "source") {
      window.sendSource(meta.label.toUpperCase());
      return;
    }

    // ---------- BLOCK NON-WEB CONTROLS ----------
    if (activeSource !== "WEB") return;

    // ---------- NORMAL EVENT / STATE ----------
    window.sendEvent(meta, name);
  };

  containers[meta.group].appendChild(btn);
  buttons[name] = btn;
  return btn;
}

// --------------------------------------------------
export function updateSource(source) {
  activeSource = source;

  // ---------- PAGE VISIBILITY ----------
  pageWeb.style.display  = source === "AUTO" ? "none"  : "block";
  pageAuto.style.display = source === "AUTO" ? "block" : "none";

  // ---------- SOURCE BUTTON HIGHLIGHT ----------
  ["JOY_CTRL", "WEB_CTRL", "AUTO_CTRL"].forEach((k) => {
    if (buttons[k]) {
      buttons[k].className = (source === k.replace("_CTRL", "")) ? "btn on" : "btn off";
    }
  });

  // ---------- DISABLE NON-WEB CONTROLS ----------
  Object.entries(buttons).forEach(([name, btn]) => {
    const meta = STATE_MAP[name];

    // safety & source su uvijek aktivni
    if (meta.group === "safety" || meta.group === "source") {
      btn.classList.remove("disabled");
      return;
    }

    if (source !== "WEB") {
      btn.classList.add("disabled");
    } else {
      btn.classList.remove("disabled");
    }
  });
}

// --------------------------------------------------
export function updateState(name, value) {
  const btn = buttons[name];
  if (!btn) return;

  // SOURCE se ne updatea preko state-a
  if (STATE_MAP[name]?.group === "source") return;

  btn.className = value === "1" ? "btn on" : "btn off";
}

// --------------------------------------------------
export function initUI() {
  Object.entries(STATE_MAP).forEach(([name, meta]) => {
    getButton(name, meta);
  });
}
