#include <Arduino.h>
#include <RemoteControl.h>
#include <LineFollower.h>

// ---------- Select mode ----------
// Manual = Bluetooth remote (hold F/B/L/R/S)
// Auto   = IR line follower (PID path correction)
// Switch via Bluetooth: send 'A' for AUTO, 'M' for MANUAL
enum class Mode : uint8_t { MANUAL, AUTO };

static RemoteControl g_remoteControl;
static LineFollower g_lineFollower; // uses IR pins 36,39,34,35,32 by default
static Mode g_mode = Mode::MANUAL;

// Optional: uncomment to auto-calibrate IR threshold at boot
// static bool g_doCalibrate = true;

void setup()
{
  Serial.begin(115200);
  // Init motors once via RemoteControl; LineFollower reuses same L298N pins/channels
  g_remoteControl.begin();
  g_lineFollower.begin(false); // false = don't re-init motors (already inited)

  // Example custom tuning:
  // LineFollower::Config cfg;
  // cfg.baseSpeed = 130;
  // cfg.kp = 0.09f; cfg.ki = 0.0f; cfg.kd = 0.18f;
  // cfg.analogThreshold = 2000;
  // g_lineFollower = LineFollower(cfg);

  // g_lineFollower.calibrate(); // place robot on line/white and auto-find threshold

  Serial.println("Ready: MANUAL (Bluetooth F/B/L/R/S). Send 'A' for AUTO line follow, 'M' for manual.");
}

void loop()
{
  // Allow mode switching via same BluetoothSerial inside RemoteControl?
  // RemoteControl doesn't expose raw BT, so we handle switch via Serial for demo.
  // For Bluetooth switch, add a method to RemoteControl or duplicate BT here.
  // Simple demo: use USB Serial to switch modes.
  if (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == 'A' || c == 'a')
    {
      g_mode = Mode::AUTO;
      Serial.println("-> AUTO line follower");
    }
    else if (c == 'M' || c == 'm')
    {
      g_mode = Mode::MANUAL;
      g_lineFollower.stop();
      Serial.println("-> MANUAL remote");
    }
    // also allow manual line debug
    if (c == 'p' || c == 'P')
    {
      g_lineFollower.printSensors(Serial);
    }
  }

  if (g_mode == Mode::AUTO)
  {
    g_lineFollower.update(); // IR detect + PID correct path
  }
  else
  {
    g_remoteControl.update(); // Bluetooth hold-to-run
  }

  delay(10U);
}
