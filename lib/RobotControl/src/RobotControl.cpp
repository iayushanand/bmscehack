#include "RobotControl.h"

namespace RobotControl
{

RobotMode g_mode = RobotMode::MANUAL;
uint8_t g_speed = 120U;
uint8_t g_maxSpeed = 200U;
uint8_t g_minSpeed = 0U;
uint8_t g_ledPin = 2U;

void begin(RobotMode mode, uint8_t speed, uint8_t maxSpeed)
{
  g_maxSpeed = maxSpeed;
  g_mode = mode;
  setSpeed(speed);
  pinMode(g_ledPin, OUTPUT);

  digitalWrite(g_ledPin, (g_mode == RobotMode::MANUAL) ? HIGH : LOW);
}

void setLedPin(uint8_t pin)
{
  g_ledPin = pin;
  pinMode(g_ledPin, OUTPUT);
}

void updateLed()
{
  if (isManual())
  {
    digitalWrite(g_ledPin, HIGH);
  }
  else
  {

    static uint32_t lastToggleMs = 0;
    static bool ledState = false;
    uint32_t now = millis();
    if (now - lastToggleMs >= 1000UL)
    {
      lastToggleMs = now;
      ledState = !ledState;
      digitalWrite(g_ledPin, ledState ? HIGH : LOW);
    }
  }
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

      static uint32_t lastToggleMs = 0;
      uint32_t now = millis();
      if (now - lastToggleMs < 400UL)
      {
        return true;
      }
      lastToggleMs = now;
      toggleMode();
      return true;
    }
    case '+':
    case 'Q':
    case 'q':
    case 'C':
    case 'c':
      changeSpeed(10);
      return true;
    case '-':
    case 'E':
    case 'e':

      changeSpeed(-10);
      return true;

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

}
