//tomo_web/html/state.js

import { SOURCE, CATEGORY, STATE, EVENT, LIGHT, SYSTEM } from "./ros_enums.js";
export const STATE_MAP = {

  // ---------- SOURCE ----------
  PS4_CTRL:  { group: "source", label: "PS4", type: "source", category: CATEGORY.SYSTEM, code: SYSTEM.FORCE_SOURCE, value: SOURCE.PS4 },
  WEB_CTRL:  { group: "source", label: "WEB", type: "source", category: CATEGORY.SYSTEM, code: SYSTEM.FORCE_SOURCE, value: SOURCE.WEB },
  AUTO_CTRL: { group: "source", label: "AUTO", type: "source", category: CATEGORY.SYSTEM, code: SYSTEM.FORCE_SOURCE, value: SOURCE.AUTO },

  // ---------- FEEDBACK SOURCE ----------
  FEEDBACK_ROS:  { group: "feedback", label: "ROS", type: "feedback" },
  FEEDBACK_ESP:  { group: "feedback", label: "ESP", type: "feedback" },

  // ---------- SAFETY ----------
  FAILSAFE:  { group: "safety", label: "Failsafe", type: "failsafe" },
  HARD_EMERGENCY:    { group: "safety", label: "HARD EMERGENCY", type: "emergency_hard" },
  SOFT_EMERGENCY:    { group: "safety", label: "SOFT EMERGENCY", type: "emergency_soft" },
  RELEASE_EMERGENCY: { group: "safety", label: "Release EMERGENCY", type: "emergency_release" },

  // ---------- STATES ----------
  ARMED: { group: "states", label: "Armed", category: CATEGORY.STATE, code: STATE.ARMED },
  POWER: { group: "states", label: "Power", category: CATEGORY.STATE, code: STATE.POWER },
  LIGHT: { group: "states", label: "Lights", category: CATEGORY.STATE, code: STATE.LIGHT },

  // ---------- EVENTS ----------
  ENGINE_START: { group: "events", label: "Engine Start", category: CATEGORY.EVENT, code: EVENT.ENGINE_START },
  ENGINE_STOP:  { group: "events", label: "Engine Stop", category: CATEGORY.EVENT, code: EVENT.ENGINE_STOP },
  CLUTCH:       { group: "events", label: "Clutch Active", category: CATEGORY.EVENT, code: EVENT.CLUTCH_ACTIVE },
  BRAKE:        { group: "events", label: "Brake Active", category: CATEGORY.EVENT, code: EVENT.BRAKE_ACTIVE},
  MOVE:         { group: "events", label: "Move Allowed", category: CATEGORY.EVENT, code: EVENT.MOVE_ALLOWED  },

  // ---------- LIGHTS ----------
  FP: { group: "lights", label: "Front Position", category: CATEGORY.LIGHT, code: LIGHT.FRONT_POSITION },
  FS: { group: "lights", label: "Front Short", category: CATEGORY.LIGHT, code: LIGHT.FRONT_SHORT },
  FL: { group: "lights", label: "Front Long", category: CATEGORY.LIGHT, code: LIGHT.FRONT_LONG },
  BP: { group: "lights", label: "Back Position", category: CATEGORY.LIGHT, code: LIGHT.BACK_POSITION },
  LB: { group: "lights", label: "Left Blink", category: CATEGORY.LIGHT, code: LIGHT.LEFT_BLINK },
  RB: { group: "lights", label: "Right Blink", category: CATEGORY.LIGHT, code: LIGHT.RIGHT_BLINK },
};

export const localState = {
  PS4_CTRL: "0",
  WEB_CTRL: "0",
  AUTO_CTRL: "0",
  FAILSAFE: "0",
};
