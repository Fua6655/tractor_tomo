//tomo_web/html/state.js

import { SOURCE, CATEGORY, STATE, EVENT, SIGNALIZATION, SYSTEM } from "./ros_enums.js";
export const STATE_MAP = {

  // ---------- SOURCE ----------
  PS4_CTRL:  { group: "source", label: "PS4", type: "source", category: CATEGORY.SYSTEM, code: SYSTEM.FORCE_SOURCE, value: SOURCE.PS4 },
  WEB_CTRL:  { group: "source", label: "WEB", type: "source", category: CATEGORY.SYSTEM, code: SYSTEM.FORCE_SOURCE, value: SOURCE.WEB },
  AUTO_CTRL: { group: "source", label: "AUTO", type: "source", category: CATEGORY.SYSTEM, code: SYSTEM.FORCE_SOURCE, value: SOURCE.AUTO },

  // ---------- SAFETY ----------
  FAILSAFE:  { group: "safety", label: "Failsafe", type: "failsafe" },
  FAILSAFE_ESP1:  { group: "safety", label: "ESP1 Failsafe", type: "failsafe_sub" },
  FAILSAFE_ESP2:  { group: "safety", label: "ESP2 Failsafe", type: "failsafe_sub" },
  HARD_EMERGENCY:    { group: "safety", label: "HARD EMERGENCY", type: "emergency_hard" },
  POWER_EMERGENCY:   { group: "safety", label: "POWER FAILSAFE", type: "emergency_power" },
  SOFT_EMERGENCY:    { group: "safety", label: "SOFT EMERGENCY", type: "emergency_soft" },
  RELEASE_EMERGENCY: { group: "safety", label: "Release EMERGENCY", type: "emergency_release" },

  // ---------- STATES ----------
  ARMED: { group: "states", label: "Armed", category: CATEGORY.STATE, code: STATE.ARMED },
  ENGINE: { group: "states", label: "Engine", category: CATEGORY.STATE, code: STATE.ENGINE },
  SIGNALIZATION: { group: "states", label: "Signalization", category: CATEGORY.STATE, code: STATE.SIGNALIZATION },
  MOVE: { group: "states", label: "Move Allowed", category: CATEGORY.STATE, code: STATE.MOVE_ALLOWED },

  // ---------- EVENTS ----------
  ENGINE_START: { group: "events", label: "Engine Start", category: CATEGORY.EVENT, code: EVENT.ENGINE_START },
  ENGINE_STOP:  { group: "events", label: "Engine Stop", category: CATEGORY.EVENT, code: EVENT.ENGINE_STOP },
  CLUTCH:       { group: "events", label: "Clutch Active", category: CATEGORY.EVENT, code: EVENT.CLUTCH_ACTIVE },
  BRAKE:        { group: "events", label: "Brake Active", category: CATEGORY.EVENT, code: EVENT.BRAKE_ACTIVE},

  // ---------- SIGNALIZATION ----------
  FP: { group: "signalization", label: "Front Position", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.FRONT_POSITION },
  FS: { group: "signalization", label: "Front Short", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.FRONT_SHORT },
  FL: { group: "signalization", label: "Front Long", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.FRONT_LONG },
  BP: { group: "signalization", label: "Back Position", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.BACK_POSITION },
  LB: { group: "signalization", label: "Left Blink", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.LEFT_BLINK },
  RB: { group: "signalization", label: "Right Blink", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.RIGHT_BLINK },
  HO: { group: "signalization", label: "Horn", category: CATEGORY.SIGNALIZATION, code: SIGNALIZATION.HORN },

};

export const localState = {
  PS4_CTRL: "0",
  WEB_CTRL: "0",
  AUTO_CTRL: "0",
  FAILSAFE: "0",
};
