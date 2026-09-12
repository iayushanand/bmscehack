#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <stdint.h>

/**
 * @brief Bluetooth Classic remote control for ESP32 + L298N dual BO motors.
 *
 * Hold-to-run protocol (send repeatedly while button is pressed):
 *   'F' -> forward
 *   'B' -> backward
 *   'L' -> left (pivot)
 *   'R' -> right (pivot)
 *   'S' -> stop immediately
 *
 * If no motion command is received within COMMAND_HOLD_TIMEOUT_MS,
 * the robot auto-stops (safety when Bluetooth/controller disconnects).
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
          holdTimeoutMs(250UL),
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

  void setSpeed(uint8_t duty);
  bool isBluetoothConnected();

private:
  void setLeftMotor(bool forward, uint8_t duty);
  void setRightMotor(bool forward, uint8_t duty);

  Config m_config;
  BluetoothSerial m_btSerial;
  volatile uint32_t m_lastMotionCommandMs = 0UL;
  bool m_initialized = false;
};
