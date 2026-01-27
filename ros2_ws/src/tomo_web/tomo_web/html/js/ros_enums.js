//tomo_web/html/ros_enums.js

// ---------- SOURCE ----------
export const SOURCE = {
  PS4: 1,
  WEB: 2,
  AUTO: 3,
};

// ---------- CATEGORY ----------
export const CATEGORY = {
  STATE: 1,
  EVENT: 2,
  LIGHT: 3,
  SYSTEM: 4,
};

// ---------- STATE ----------
export const STATE = {
  ARMED: 1,
  POWER: 2,
  LIGHT: 3,
};

// ---------- EVENT ----------
export const EVENT = {
  ENGINE_START: 10,
  ENGINE_STOP: 11,
  CLUTCH_ACTIVE: 12,
  BRAKE_ACTIVE: 13,
  MOVE_ALLOWED: 14,
};

// ---------- LIGHT ----------
export const LIGHT = {
  FRONT_POSITION: 20,
  FRONT_SEQUENCE_NEXT: 21,
  FRONT_SHORT: 22,
  FRONT_LONG: 23,
  BACK_POSITION: 24,
  LEFT_BLINK: 25,
  RIGHT_BLINK: 26,
};

// ---------- SYSTEM ----------
export const SYSTEM = {
  FORCE_SOURCE: 40,
  RESET: 41,
};
