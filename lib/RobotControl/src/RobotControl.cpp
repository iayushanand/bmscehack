#include "RobotControl.h"

namespace RobotControl
{

// MANUAL / LINE_FOLLOWER
RobotMode g_mode = RobotMode::MANUAL;
uint8_t g_speed = 120U;
uint8_t g_maxSpeed = 200U;
uint8_t g_minSpeed = 0U;

void begin(RobotMode mode, uint8_t speed, uint8_t maxSpeed)
{
  g_maxSpeed = maxSpeed;
  g_mode = mode;
  setSpeed(speed); // clamps
}

void setMode(RobotMode mode)
{
  if (g_mode != mode)
  {
    g_mode = mode;
  }
}

RobotMode getMode()
{
  return g_mode;
}

void toggleMode()
{
  g_mode = (g_mode == RobotMode::MANUAL) ? RobotMode::LINE_FOLLOWER : RobotMode::MANUAL;
}

bool isManual()
{
  return g_mode == RobotMode::MANUAL;
}

bool isLineFollower()
{
  return g_mode == RobotMode::LINE_FOLLOWER;
}

const char *modeToString(RobotMode mode)
{
  return (mode == RobotMode::MANUAL) ? "MANUAL" : "LINE_FOLLOWER";
}

const char *modeToString()
{
  return modeToString(g_mode);
}

void setSpeed(uint8_t speed)
{
  if (speed > g_maxSpeed) speed = g_maxSpeed;
  if (speed < g_minSpeed) speed = g_minSpeed;
  g_speed = speed;
}

uint8_t getSpeed()
{
  return g_speed;
}

void changeSpeed(int8_t delta)
{
  int16_t next = (int16_t)g_speed + delta;
  if (next < (int16_t)g_minSpeed) next = g_minSpeed;
  if (next > (int16_t)g_maxSpeed) next = g_maxSpeed;
  g_speed = (uint8_t)next;
}

void setMaxSpeed(uint8_t max)
{
  g_maxSpeed = max;
  if (g_speed > g_maxSpeed) g_speed = g_maxSpeed;
}

void setMinSpeed(uint8_t min)
{
  g_minSpeed = min;
  if (g_speed < g_minSpeed) g_speed = g_minSpeed;
}

uint8_t getMaxSpeed()
{
  return g_maxSpeed;
}

uint8_t getMinSpeed()
{
  return g_minSpeed;
}

bool handleCommand(char c)
{
  switch (c)
  {
    case 'A':
    case 'a':
      setMode(RobotMode::LINE_FOLLOWER);
      return true;
    case 'M':
    case 'm':
      setMode(RobotMode::MANUAL);
      return true;
    case 'T':
    case 't':
    case 'X':
    case 'x':
    {
      // Debounce toggle (controller hold-to-repeat sends X repeatedly)
      static uint32_t lastToggleMs = 0;
      uint32_t now = millis();
      if (now - lastToggleMs < 400UL)
      {
        return true; // consume but ignore rapid repeat
      }
      lastToggleMs = now;
      toggleMode();
      return true;
    }
    case '+':
    case 'U':
    case 'u':
      changeSpeed(10);
      return true;
    case '-':
    case 'D':
    case 'd':
      changeSpeed(-10);
      return true;
    case '0': setSpeed(0); return true;
    case '1': setSpeed(28); return true;
    case '2': setSpeed(56); return true;
    case '3': setSpeed(84); return true;
    case '4': setSpeed(112); return true;
    case '5': setSpeed(140); return true;
    case '6': setSpeed(168); return true;
    case '7': setSpeed(196); return true;
    case '8': setSpeed(220); return true;
    case '9': setSpeed(252); return true;
    default:
      return false;
  }
}

void printState(Stream &out)
{
  out.print("[RobotControl] mode=");
  out.print(modeToString());
  out.print(" speed=");
  out.print(g_speed);
  out.print(" (min=");
  out.print(g_minSpeed);
  out.print(" max=");
  out.print(g_maxSpeed);
  out.println(")");
}

} // namespace RobotControl
