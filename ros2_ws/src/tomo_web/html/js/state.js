export const STATE_MAP = {

  // ---------- SOURCE ----------
  WEB_CTRL:  { group: "source", label: "Web Control", type: "source" },
  AUTO_CTRL: { group: "source", label: "Auto Control", type: "source" },

  // ---------- SAFETY ----------
  FAILSAFE:  { group: "safety", label: "Failsafe", type: "failsafe" },
  EMERGENCY: { group: "safety", label: "EMERGENCY", type: "emergency" },

  // ---------- STATES (READ ONLY) ----------
  ARMED: { group: "states", label: "Armed", type: "indicator" },
  POWER: { group: "states", label: "Power Mode", type: "indicator" },
  LIGHT: { group: "states", label: "Lights Mode", type: "indicator" },

  // ---------- EVENTS ----------
  ENGINE: { group: "events", label: "Start Engine" },
  CLUTCH: { group: "events", label: "Clutch Down" },
  SPEED:  { group: "events", label: "High Speed" },
  MOVE:   { group: "events", label: "Move Allowed" },

  // ---------- LIGHTS ----------
  FP: { group: "lights", label: "Front Position" },
  FS: { group: "lights", label: "Front Short" },
  FL: { group: "lights", label: "Front Long" },
  BACK: { group: "lights", label: "Back" },
  LB: { group: "lights", label: "Left Blink" },
  RB: { group: "lights", label: "Right Blink" },
};

export const localState = {
  WEB_CTRL: "0",
  AUTO_CTRL: "0",
  FAILSAFE: "0",
  EMERGENCY: "0",
  ARMED: "0",
  POWER: "0",
  LIGHT: "0",
};