#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <stdint.h>

/**
 * @brief Bluetooth Classic remote control for ESP32 + L298N dual BO motors.
 * Gamepad-compatible (Arduino Bluetooth Controller app sends btn + trailing '0').
 *
 * Press-and-hold (holdTimeout=0 in main): press F/B/L/R to move, release to stop.
 * Release is trailing '0' from gamepad (see log: 'C'+'0', 'S'+'0', 'F'+'0').
 *   'F'/'U' -> forward  (GamePad Up)
 *   'B'/'D' -> backward (GamePad Down)
 *   'L'     -> left pivot (reduced power via turnScale)
 *   'R'     -> right pivot (reduced power via turnScale)
 *   'G' -> forward-left diagonal, 'I' -> forward-right
 *   'H' -> backward-left, 'J' -> backward-right (BT RC Controller diagonals)
 *   '0' (or 0x00) -> RELEASE: stop motors, keep speed (ignored <250ms after C/E speed change)
 *   'C' (Circle): speed +10 persistent, instant even while moving
 *   'S' (Square) dual: STOP keep-speed when moving (D-pad release),
 *     speed -10 when already stopped. In AUTO always -10.
 *   'E'/'-': speed -10 anytime even while moving (use this to slow without stopping)
 *   'Z' -> emergency STOP (motors off, keeps speed value)
 *   'X' (Cross) -> toggle MANUAL <-> LINE_FOLLOWER (handled via RobotControl)
 *   '+'/'Q' -> speed +10 (global via RobotControl)
 */
class RemoteControl
{
public:
  struct Config
  {
    uint8_t pinLeftIn1;
    uint8_t pinLeftIn2;
    uint8_t pinLeftEn;
    uint8_t pinRightIn1;
    uint8_t pinRightIn2;
    uint8_t pinRightEn;
    uint8_t pwmChannelLeft;
    uint8_t pwmChannelRight;
    uint32_t pwmFreqHz;
    uint8_t pwmResolutionBits;
    uint8_t motorSpeedDuty; // 0..255
    uint32_t holdTimeoutMs;
    uint8_t turnScalePct; // 0..100 power for pivot L/R turns (60 = softer turns)
    const char *btDeviceName;

    Config()
        : pinLeftIn1(26U),
          pinLeftIn2(27U),
          pinLeftEn(25U),
          pinRightIn1(14U),
          pinRightIn2(12U),
          pinRightEn(33U),
          pwmChannelLeft(0U),
          pwmChannelRight(1U),
          pwmFreqHz(1000UL),
          pwmResolutionBits(8U),
          motorSpeedDuty(100U),
          holdTimeoutMs(800UL),
          turnScalePct(60U),
          btDeviceName("BURNT_TOASTER")
    {
    }
  };

  explicit RemoteControl(const Config &config);
  RemoteControl();

  /** Initialize GPIOs, PWM and Bluetooth Serial. Call from setup(). */
  void begin();

  /** Overload to override BT name / speed at runtime. */
  void begin(const char *btDeviceName, uint8_t motorSpeedDuty = 100U);

  /** Poll Bluetooth, handle commands and hold-timeout. Call from loop(). */
  void update();

  /** Handle a single command character. */
  void handleCommand(char command);

  // Direct motion primitives
  void stop();
  void forward();
  void backward();
  void left();
  void right();
  void forwardLeft();   // diagonal: left slower
  void forwardRight();  // diagonal: right slower
  void backwardLeft();
  void backwardRight();

  void setSpeed(uint8_t duty);
  uint8_t getSpeed() const { return m_config.motorSpeedDuty; }
  void setHoldTimeout(uint32_t ms) { m_config.holdTimeoutMs = ms; }
  uint32_t getHoldTimeout() const { return m_config.holdTimeoutMs; }
  void setTurnScale(uint8_t pct) { m_config.turnScalePct = (pct > 100 ? 100 : pct); }
  bool isBluetoothConnected();

  // Bluetooth passthrough for RobotControl mode/speed handling
  bool btAvailable();
  int btRead();
  Stream &getBTStream();
  void printBTInfo(Stream &out = Serial);

private:
  void setLeftMotor(bool forward, uint8_t duty);
  void setRightMotor(bool forward, uint8_t duty);
  void reapplyLastMotion(); // re-drive stored motion at new speed (instant speed change)

  Config m_config;
  BluetoothSerial m_btSerial;
  volatile uint32_t m_lastMotionCommandMs = 0UL;
  bool m_initialized = false;
  char m_lastMotionCmd = 'S'; // last latch motion: F/B/L/R/G/I/H/J, 'S' = stopped
  uint32_t m_lastSpeedChangeMs = 0UL; // guard trailing '0' after C/E speed buttons
};
