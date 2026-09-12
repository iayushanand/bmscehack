#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Global robot state: mode switch + shared speed.
 *
 * Single source of truth for MANUAL (Bluetooth RemoteControl)
 * and LINE_FOLLOWER (IR + PID).
 *
 *   RobotControl::g_mode  -> current RobotMode
 *   RobotControl::g_speed -> global base speed 0..255 used by BOTH subsystems
 *
 * All libs read/write through RobotControl so speed stays in sync.
 *
 * Usage:
 *   RobotControl::begin(RobotMode::MANUAL, 120);
 *   RobotControl::setSpeed(150); // updates global, then propagate:
 *   g_remoteControl.setSpeed(RobotControl::getSpeed());
 *   g_lineFollower.setBaseSpeed(RobotControl::getSpeed());
 *   RobotControl::setMode(RobotMode::LINE_FOLLOWER);
 *   if (RobotControl::isManual()) { ... }
 */
enum class RobotMode : uint8_t
{
  MANUAL = 0,       // Bluetooth hold-to-run via RemoteControl
  LINE_FOLLOWER = 1 // IR autonomous via LineFollower
};

namespace RobotControl
{
  // ---- Global variables (extern, defined in .cpp) ----
  extern RobotMode g_mode;
  extern uint8_t g_speed;    // 0..255 shared base speed
  extern uint8_t g_maxSpeed; // clamp upper, default 200
  extern uint8_t g_minSpeed; // clamp lower, default 0

  /** Init globals. Call from setup(). */
  void begin(RobotMode mode = RobotMode::MANUAL, uint8_t speed = 120U, uint8_t maxSpeed = 200U);

  // ---- Mode API ----
  void setMode(RobotMode mode);
  RobotMode getMode();
  void toggleMode();
  bool isManual();
  bool isLineFollower();
  const char *modeToString(RobotMode mode);
  const char *modeToString(); // current

  // ---- Speed API (global) ----
  void setSpeed(uint8_t speed);          // clamp 0..g_maxSpeed
  uint8_t getSpeed();
  void changeSpeed(int8_t delta);        // e.g. +10 / -10, clamped
  void setMaxSpeed(uint8_t max);
  void setMinSpeed(uint8_t min);
  uint8_t getMaxSpeed();
  uint8_t getMinSpeed();

  /**
   * @brief Handle single char command for mode/speed (returns true if handled).
   * Supported:
   *   'A'/'a' -> LINE_FOLLOWER
   *   'M'/'m' -> MANUAL
   *   'T'/'t' / 'X'/'x' -> toggle (X button on controller, debounced 400ms)
   *   '+' / 'U'/'u' -> speed +10
   *   '-' / 'D'/'d' -> speed -10
   *   '0'..'9' -> speed = digit*28 (~0..252)
   *   'S'/'s' -> stop handled elsewhere, but returns false here
   */
  bool handleCommand(char c);

  /** Print current state to stream. */
  void printState(Stream &out = Serial);
} // namespace RobotControl
