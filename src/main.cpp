#include <Arduino.h>
#include "BluetoothSerial.h"

// Check Bluetooth is enabled in SDK
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Run `make menuconfig` to enable it
#endif

/*
 * ESP32 + Smartphone Bluetooth -> DC Motor Control
 * ------------------------------------------------
 * Board: esp32dev (Arduino framework)
 * Bluetooth: Classic SPP (BluetoothSerial) - pairs like HC-05
 * Device name: "ESP32_Motor" - search this on your phone
 * App: "Serial Bluetooth Terminal" (Android) or any BT terminal
 *      iOS needs BLE - see note at bottom.
 *
 * Motor Driver Wiring (L298N / L293D / TB6612 / MX1508)
 * ------------------------------------------------------
 * ESP32 GPIO 27  -> IN1 / AIN1
 * ESP32 GPIO 26  -> IN2 / AIN2
 * ESP32 GPIO 14  -> ENA / PWMA (PWM speed, optional - jumpers if not used)
 * ESP32 GND      -> Driver GND (COMMON GND MANDATORY)
 * Driver VCC     -> 5V or battery (depending on driver)
 * Driver VM/VCC  -> Motor power supply (e.g. 6-12V for L298N)
 * Driver OUT1/OUT2 -> Motor terminals
 * Motor power GND -> ESP32 GND (common ground!)
 *
 * Commands (send single char from phone):
 *   F / f / 1 -> Forward
 *   B / b / 2 -> Backward
 *   S / s / 0 -> Stop (coast)
 *   X / x     -> Brake (short brake, both HIGH)
 *   3-9       -> Forward at different speed (3=low, 9=max)
 *   q,w,e...  -> Backward at different speed (if you want)
 *   V         -> Report status
 *
 * Adjust MOTOR_IN1 / MOTOR_IN2 / MOTOR_ENA to match your wiring.
 */

// ---------- Pin Config ----------
#define LED_BUILTIN 2
#define MOTOR_IN1 27
#define MOTOR_IN2 26
#define MOTOR_ENA 14   // PWM pin - set to -1 if you don't use ENA (jumper on L298N)

// PWM config for ENA
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8 // 0-255

// Speed
static uint8_t currentSpeed = 200; // 0-255 default

BluetoothSerial SerialBT;
String deviceName = "ESP32_Motor";

// ---------- Motor Helpers ----------
inline void pwmWriteENA(uint8_t val) {
#if MOTOR_ENA != -1
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_ENA, val);
#else
  ledcWrite(PWM_CHANNEL, val);
#endif
#endif
}

void motorStop() {
  pwmWriteENA(0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  Serial.println("[MOTOR] STOP (coast)");
  if (SerialBT.hasClient()) SerialBT.println("STOP");
}

void motorBrake() {
  pwmWriteENA(255);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  Serial.println("[MOTOR] BRAKE");
  if (SerialBT.hasClient()) SerialBT.println("BRAKE");
}

void motorForward(uint8_t speed = 255) {
  currentSpeed = speed;
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  pwmWriteENA(speed);
  Serial.printf("[MOTOR] FORWARD speed=%d\n", speed);
  if (SerialBT.hasClient()) SerialBT.printf("FORWARD %d\n", speed);
}

void motorBackward(uint8_t speed = 255) {
  currentSpeed = speed;
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  pwmWriteENA(speed);
  Serial.printf("[MOTOR] BACKWARD speed=%d\n", speed);
  if (SerialBT.hasClient()) SerialBT.printf("BACKWARD %d\n", speed);
}

void handleCommand(char cmd) {
  // trim whitespace already
  switch (cmd) {
    case 'F':
    case 'f':
    case '1':
      motorForward(currentSpeed);
      break;
    case 'B':
    case 'b':
    case '2':
      motorBackward(currentSpeed);
      break;
    case 'S':
    case 's':
    case '0':
      motorStop();
      break;
    case 'X':
    case 'x':
      motorBrake();
      break;
    // numeric speed control 3-9 for forward
    case '3': motorForward(80); break;
    case '4': motorForward(120); break;
    case '5': motorForward(160); break;
    case '6': motorForward(200); break;
    case '7': motorForward(220); break;
    case '8': motorForward(240); break;
    case '9': motorForward(255); break;
    case 'V':
    case 'v':
      Serial.printf("Status: IN1=%d IN2=%d Speed=%d\n",
                    digitalRead(MOTOR_IN1), digitalRead(MOTOR_IN2), currentSpeed);
      if (SerialBT.hasClient()) SerialBT.printf("IN1=%d IN2=%d Speed=%d\n",
                    digitalRead(MOTOR_IN1), digitalRead(MOTOR_IN2), currentSpeed);
      break;
    default:
      Serial.printf("[BT] Unknown cmd: '%c' (%d)\n", cmd, cmd);
      if (SerialBT.hasClient()) SerialBT.printf("Unknown: %c - Use F/B/S/X or 3-9\n", cmd);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ESP32 Bluetooth Motor Control ---");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

#if MOTOR_ENA != -1
  pinMode(MOTOR_ENA, OUTPUT);
  // Arduino-ESP32 v2.x vs v3.x LEDC API handling
  // Try new API first (ledcAttach), fallback to old (ledcSetup/ledcAttachPin)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_ENA, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_ENA, PWM_CHANNEL);
#endif
  pwmWriteENA(0);
#endif

  // Start Bluetooth Classic
  if (!SerialBT.begin(deviceName)) {
    Serial.println("ERROR: Bluetooth init failed! Check CONFIG_BT_ENABLED");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }
  Serial.printf("Bluetooth Started! Pair with \"%s\" on your phone\n", deviceName.c_str());
  Serial.println("Commands: F=Forward B=Backward S=Stop X=Brake 3..9=Speed V=Status");
  SerialBT.println("ESP32 Motor Ready. Send F/B/S");
}

void loop() {
  // --- Bluetooth -> Motor ---
  if (SerialBT.available()) {
    char c = SerialBT.read();
    // ignore newline/carriage return but process buffer
    if (c == '\n' || c == '\r' || c == ' ') return;
    Serial.printf("[BT RX] %c\n", c);
    handleCommand(c);
  }

  // --- USB Serial -> Bluetooth (for testing from Serial Monitor) ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') return;
    Serial.printf("[USB RX] %c -> BT\n", c);
    handleCommand(c); // also control locally
    if (SerialBT.hasClient()) SerialBT.printf("Echo: %c\n", c);
  }

  // LED indicates Bluetooth connection status
  static uint32_t lastBlink = 0;
  static bool ledState = false;
  uint32_t interval = SerialBT.hasClient() ? 1000 : 200; // fast blink = waiting, slow = connected
  if (millis() - lastBlink > interval) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
}

/*
 * iOS NOTE:
 * iPhone does NOT support Bluetooth Classic SPP. For iOS use BLE:
 * Replace BluetoothSerial with NimBLE (Arduino BLE) and create a service
 * with a writable characteristic. Apps like "nRF Connect" or "LightBlue"
 * can then send F/B/S. Let me know if you need the BLE version.
 *
 * L298N without ENA jumper:
 * If your L298N has ENA jumper cap ON, you don't need MOTOR_ENA wiring
 * and motor will run at full speed. Set MOTOR_ENA to -1 or remove PWM code.
 */
