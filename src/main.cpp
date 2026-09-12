#include <Arduino.h>
#include <RobotControl.h>
#include <RemoteControl.h>
#include <LineFollower.h>

// Global instances - motors share same L298N pins/channels (26/27/25/14/12/33, ch 0/1)
// RobotControl holds single source of truth: g_mode + g_speed
// LineFollower lib owns everything line-related: 2xIR (36/39) black line (HIGH=line),
// fixed straight 80 / turn 100, 10ms AUTO burst (see lib/LineFollower/src/LineFollower.h)
static RemoteControl g_remoteControl;
static LineFollower g_lineFollower; // lib defaults: 2 sensors, black line HIGH, fixed 80/100

// Mode edge detector (AUTO burst itself lives in LineFollower::startAuto/update)
static RobotMode s_prevMode = RobotMode::MANUAL;

void setup()
{
  Serial.begin(115200);

  // Shared speed 100 default (controller Circle/Square adjust both manual + line live).
  // Line follower: straight 100 / pivot 80, follows global (turn scales proportionally).
  RobotControl::begin(RobotMode::MANUAL, 100U, 200U);

  // Init motors once via RemoteControl; LineFollower reuses same pins
  g_remoteControl.begin();      // syncs motorSpeedDuty from RobotControl::g_speed (100)
  g_remoteControl.setHoldTimeout(0); // press-and-hold: press F/B/L/R to move, release (0) to stop
  g_lineFollower.begin(false);  // false = don't re-init motors (line speeds fixed 80/100 from lib)

  Serial.println("=== LineFollower (2xIR black line, 100/80 adjustable) + Remote ===");
  RobotControl::printState(Serial);
  Serial.println("Line: 2xIR L=36 R=39 black-line (HIGH=line), straight 100 pivot 80, Circle/Square adjust live.");
  Serial.println("IR values stream every loop; all white -> motors STOP.");
  Serial.println("GamePad (Arduino Bluetooth Controller):");
  Serial.println("  D-Pad Up/U/F=forward Down/D/B=back Left=L Right=R G/I/H/J=diagonals Z=stop");
  Serial.println("  X (Cross)/T -> TOGGLE MANUAL<->AUTO  A->AUTO M->MANUAL");
  Serial.println("  C (Circle)/Q/+ = +10, E/- = -10, S (Square): STOP when moving / -10 when stopped");
  Serial.println("  Speed buttons affect BOTH modes live (shared global). AUTO burst 10ms.");
  Serial.println("  USB: X/T/A/M/C/Q/E/S/Z/P also work");
}

void loop()
{
  // ---- Handle USB Serial for global mode/speed (works in BOTH modes) ----
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == 'P' || c == 'p')
    {
      g_lineFollower.printSensors(Serial);
      RobotControl::printState(Serial);
      g_remoteControl.printBTInfo(Serial);
      continue;
    }
    // Try global handle first (mode + C/Q/E speed work in AUTO too, live effect)
    bool wasGlobal = RobotControl::handleCommand(c);
    if (wasGlobal)
    {
      // Propagate shared global speed to both (line update() also auto-syncs every loop)
      g_remoteControl.setSpeed(RobotControl::getSpeed());
      g_lineFollower.setBaseSpeed(RobotControl::getSpeed());

      Serial.print("-> ");
      RobotControl::printState(Serial);

      // Safety stop on mode switch
      if (RobotControl::isManual())
      {
        g_lineFollower.stop();
      }
      else
      {
        g_remoteControl.stop();
      }
      continue;
    }
    // Otherwise forward to RemoteControl motion if in MANUAL
    if (RobotControl::isManual())
    {
      g_remoteControl.handleCommand(c);
    }
    else
    {
      // In AUTO, allow S/Z via RemoteControl even from USB (affects MANUAL speed only)
      if (c == 'S' || c == 's' || c == 'Z' || c == 'z')
      {
        g_remoteControl.handleCommand(c);
      }
    }
  }

  // ---- Always poll BT (critical: also in AUTO so X toggles back + speed works in AUTO) ----
  // C/Q/+/-10, E/-10, S(-10), X/A/M work in AUTO via intercept; motion F/B/L/R ignored in AUTO.
  g_remoteControl.update();

  // ---- MANUAL -> AUTO edge: delegate burst to LineFollower lib ----
  RobotMode curMode = RobotControl::getMode();
  if (s_prevMode == RobotMode::MANUAL && curMode == RobotMode::LINE_FOLLOWER)
  {
    uint8_t spd = RobotControl::getSpeed(); // controller-adjustable, default 70
    g_remoteControl.stop(); // ensure clean start
    g_lineFollower.startAuto(); // 10ms forward nudge, then PID (inside lib)
    Serial.print("-> AUTO line follow at speed ");
    Serial.println(spd);
    g_remoteControl.getBTStream().print("AUTO speed ");
    g_remoteControl.getBTStream().println(spd);
  }
  else if (s_prevMode == RobotMode::LINE_FOLLOWER && curMode == RobotMode::MANUAL)
  {
    g_lineFollower.cancelAuto(); // abort burst if toggled back mid-burst
    Serial.println("-> MANUAL");
  }
  s_prevMode = curMode;

  // ---- Onboard LED: MANUAL=ON, AUTO=1s blink ----
  RobotControl::updateLed();

  // ---- Run active mode (burst + PID fully inside LineFollower::update) ----
  if (RobotControl::isLineFollower())
  {
    g_lineFollower.update();
  }
  else
  {
    // MANUAL: BT already polled above.
    if (RobotControl::isLineFollower())
    {
      g_remoteControl.stop();
    }
  }

  delay(10U);
}
