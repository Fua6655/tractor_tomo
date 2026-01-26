export const STATE_MAP = {

  // ---------- SOURCE ----------
  JOY_CTRL:  { group: "source", label: "Joy Control", type: "source" },
  WEB_CTRL:  { group: "source", label: "Web Control", type: "source" },
  AUTO_CTRL: { group: "source", label: "Auto Control", type: "source" },

  // ---------- SAFETY ----------
  FAILSAFE:  { group: "safety", label: "Failsafe", type: "failsafe" },
  HARD_EMERGENCY:    { group: "safety", label: "HARD EMERGENCY", type: "emergency_hard" },
  SOFT_EMERGENCY:    { group: "safety", label: "SOFT EMERGENCY", type: "emergency_soft" },
  RELEASE_EMERGENCY: { group: "safety", label: "Release EMERGENCY", type: "emergency_release" },

  // ---------- STATES ----------
  ARMED: { group: "states", label: "Armed" },
  POWER: { group: "states", label: "Power" },
  LIGHT: { group: "states", label: "Lights" },

  // ---------- EVENTS ----------
  ENGINE_START: { group: "events", label: "Engine Start" },
  ENGINE_STOP:  { group: "events", label: "Engine Stop" },
  CLUTCH:       { group: "events", label: "Clutch Active" },
  BRAKE:        { group: "events", label: "Brake Active" },
  MOVE:         { group: "events", label: "Move Allowed" },

  // ---------- LIGHTS ----------
  FP: { group: "lights", label: "Front Position" },
  FS: { group: "lights", label: "Front Short" },
  FL: { group: "lights", label: "Front Long" },
  BP: { group: "lights", label: "Back Position" },
  LB: { group: "lights", label: "Left Blink" },
  RB: { group: "lights", label: "Right Blink" },
};

export const localState = {
  JOY_CTRL: "0",
  WEB_CTRL: "0",
  AUTO_CTRL: "0",
  FAILSAFE: "0",
};
