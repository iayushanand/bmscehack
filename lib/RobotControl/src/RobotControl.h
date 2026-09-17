#pragma once

#include <Arduino.h>
#include <stdint.h>

enum class RobotMode : uint8_t
{
  MANUAL = 0,
  LINE_FOLLOWER = 1
};

namespace RobotControl
{

  extern RobotMode g_mode;
  extern uint8_t g_speed;
  extern uint8_t g_maxSpeed;
  extern uint8_t g_minSpeed;
  extern uint8_t g_ledPin;

  void begin(RobotMode mode = RobotMode::MANUAL, uint8_t speed = 120U, uint8_t maxSpeed = 200U);
  void setLedPin(uint8_t pin);
  void updateLed();

  void setMode(RobotMode mode);
  RobotMode getMode();
  void toggleMode();
  bool isManual();
  bool isLineFollower();
  const char *modeToString(RobotMode mode);
  const char *modeToString();

  void setSpeed(uint8_t speed);
  uint8_t getSpeed();
  void changeSpeed(int8_t delta);
  void setMaxSpeed(uint8_t max);
  void setMinSpeed(uint8_t min);
  uint8_t getMaxSpeed();
  uint8_t getMinSpeed();

  bool handleCommand(char c);

  void printState(Stream &out = Serial);
}
