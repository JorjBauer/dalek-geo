#include <Arduino.h>
#include <SPI.h>
#include <DAC_MCP49xx.h>
#include <PID_v1.h>
#include "Movement.h"

//#define DEBUG

#define MCP4921_LDAC_PIN 3
#define MCP4921_SS_PIN 10

// Is the brake on by default when the system boots?
#define DEFAULTBRAKE 0

DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, MCP4921_SS_PIN, MCP4921_LDAC_PIN);

Movement movement; // the movement control engine code...

#define LsensorA 16
#define LsensorB 15
#define LsensorC 17
#define LsensorPower 14

#define RsensorA 8
#define RsensorB 7
#define RsensorC 6
#define RsensorPower 9

#define Motor1DirectionPin 21
#define Motor1BrakePin 20
#define Motor2DirectionPin 23
#define Motor2BrakePin 22

#define bitShiftModifier 7

#define mcpShutdown 2

unsigned long motorTimer = 0;
unsigned long brakeTimer = 0;

#define kFORWARD 1
#define kBACKWARD -1
#define kZERO 0

// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200

void setTimer(unsigned long *t)
{
  *t = millis() + FLOAT_TIME;
}


void setup() {
  pinMode(mcpShutdown, OUTPUT);
  digitalWrite(mcpShutdown, 1);

  pinMode(LsensorA, INPUT);
  pinMode(LsensorB, INPUT);
  pinMode(LsensorC, INPUT);
  pinMode(LsensorPower, INPUT);

  pinMode(RsensorA, INPUT);
  pinMode(RsensorB, INPUT);
  pinMode(RsensorC, INPUT);
  pinMode(RsensorPower, INPUT);

  pinMode(Motor1DirectionPin, INPUT);
  pinMode(Motor1BrakePin, OUTPUT);
  pinMode(Motor2DirectionPin, INPUT);
  pinMode(Motor2BrakePin, OUTPUT);

  digitalWrite(Motor1BrakePin, DEFAULTBRAKE);
  digitalWrite(Motor2BrakePin, DEFAULTBRAKE);

  Serial.begin(115200);
  Serial.println("Startup");
  Serial1.begin(57600); // slower b/c there's a softwareSerial running on the Moteino

  SPI.begin();

  dac.outputA(0); // 0 - 4095
  dac.outputB(0);
  dac.latch();

  delay(1000); // startup of the downstream controllers?
}

void setBrakePins(bool leftBrake, bool isOn)
{
  static bool lastLeftBrake = DEFAULTBRAKE;
  static bool lastRightBrake = DEFAULTBRAKE;

  if (leftBrake) {
    if (lastLeftBrake != isOn) {
      Serial.print("LB: ");
      Serial.print(isOn? "on" : "off");
      Serial.print(" rb: ");
      Serial.println(lastRightBrake? "on" : "off");
      digitalWrite(Motor2BrakePin, isOn ? HIGH : LOW);
      lastLeftBrake = isOn;
    }
  } else {
    if (lastRightBrake != isOn) {
      Serial.print("lb: ");
      Serial.print(lastLeftBrake? "on" : "off");
      Serial.print(" RB: ");
      Serial.println(isOn? "on" : "off");
      digitalWrite(Motor1BrakePin, isOn ? HIGH : LOW);
      lastRightBrake = isOn;
    }
  }
}

void setBrake(bool isOn) 
{
  movement.Brake(isOn);
  // set the actual brake pins immediately
  setBrakePins(false, isOn);
  setBrakePins(true, isOn);
}

// Is it ok for us to set the motor to 'wantSetTo' if it was last set to 'lastSetTo'?
// return true if so, or false to abort.
bool sanityCheckMotor(int16_t lastSetTo, int16_t wantSetTo)
{
  int signLast = sign(lastSetTo);
  int signWant = sign(wantSetTo);

  // Anything is ok if it was off to begin with. (FIXME: don't try to go too fast.)
  if (signLast == kZERO)
    return true;

  // If we want to stop, that's okay.
  if (signWant == kZERO)
    return true;

  // Otherwise, expect that we're going in the same direction as we want to.
  if (signLast != signWant)
    return false;

  return true;
}

inline int8_t sign(int16_t v)
{
  if (v < 0)
    return kBACKWARD;

  if (v > 0)
    return kFORWARD;

  return kZERO;
}

void updateMotors()
{
  static int16_t lastLeftMotor = 0;
  static int16_t lastRightMotor = 0;

  bool decelLeft, decelRight;
  movement.GetBrakeState(&decelLeft, &decelRight);
#ifdef DEBUG
  static bool lastDL = false;
  if (lastDL != decelLeft) {
    Serial.print("BL: ");
    Serial.println(decelLeft ? 1 : 0);
    lastDL = decelLeft;
  }
#endif
  setBrakePins(true, decelLeft);
  setBrakePins(false, decelRight);

  int16_t current_left_motor, current_right_motor;
  movement.GetMotorValues(&current_left_motor,
			  &current_right_motor);

  // The controllers need to be told to go in to reverse *before* we go in to reverse.
  // So it's possible that Movement will tell us we're moving @ 0, but that we also need
  // to have reverse engaged - so that it happens before we start trying to move.
  int8_t currentLeftDir, currentRightDir;
  movement.GetMotorDirection(&currentLeftDir, &currentRightDir);

  if (!sanityCheckMotor(lastLeftMotor, current_left_motor) ||
      !sanityCheckMotor(lastRightMotor, current_right_motor)) {
#ifdef DEBUG
    Serial.println(" ACCEL ERR");
#endif
    movement.Reset();
    dac.outputA(0);
    dac.outputB(0);
    dac.latch();
    return; // refuse to perfrom the update; shut down both motors instead.
  }

  static int8_t lastLeftMotorDir = kZERO;
  static int8_t lastRightMotorDir = kZERO;

  if (lastLeftMotorDir != currentLeftDir) {
    lastLeftMotorDir = currentLeftDir;
    if (lastLeftMotorDir == kBACKWARD) {
#ifdef DEBUG
      Serial.println(" rev");
#endif

      pinMode(Motor1DirectionPin, OUTPUT);
      digitalWrite(Motor1DirectionPin, LOW); // drag to ground
    } else {
#ifdef DEBUG
      Serial.println(" fwd");
#endif
      pinMode(Motor1DirectionPin, INPUT);
      digitalWrite(Motor1DirectionPin, HIGH);
    }
  }

  if (lastRightMotorDir != currentRightDir) {
    lastRightMotorDir = currentRightDir;
    if (lastRightMotorDir == kBACKWARD) {
      pinMode(Motor2DirectionPin, OUTPUT);
      digitalWrite(Motor2DirectionPin, LOW); // drag to ground
    } else {
      pinMode(Motor2DirectionPin, INPUT);
      digitalWrite(Motor2DirectionPin, HIGH);
    }
  }

  bool didUpdate = false;
  if (current_left_motor != lastLeftMotor) {
    lastLeftMotor = current_left_motor;
#ifdef DEBUG
    Serial.print("cl: ");
    Serial.print(current_left_motor);
    Serial.print(" ll: ");
    Serial.println(lastLeftMotor);
#endif
    
    dac.outputB(abs(lastLeftMotor));
    didUpdate = true;
  }
  if (current_right_motor != lastRightMotor) {
    lastRightMotor = current_right_motor;
    dac.outputA(abs(lastRightMotor));
    didUpdate = true;
  }
  if (didUpdate)
    dac.latch();
}


void loop() {
  static uint8_t ctr = 0;
  if (++ctr == 0) {
    movement.Update();
  }

  updateMotors();

  // safety...
  if (motorTimer && motorTimer < millis()) {
    motorTimer = 0;
    setTimer(&brakeTimer);
    movement.JoystickTankTarget(0,0);
    setBrake(true);
  }

  /* Deal with brakes the same way */
  if (brakeTimer && brakeTimer < millis()) {
    brakeTimer = 0;
    setBrake(false); // turn off the brake
  }

  if (Serial1.available()) {
    uint8_t c = Serial1.read();

    if (c == '+') {
#ifdef DEBUG
      Serial.println("+");
#endif
      movement.setFastMode(true);
    } else if (c == '-') {
#ifdef DEBUG
      Serial.println("-");
#endif
      movement.setFastMode(false);
    } else if (c == 'm') {
      setTimer(&brakeTimer);
      setBrake(true);
    }
    else if (c == 'T') {
      if (Serial1.available() >= 2) {
	int8_t tl = Serial1.read(); // -100 to 100
	int8_t tr = Serial1.read(); // -100 to 100
	movement.JoystickTankTarget(tl, tr);
	setTimer(&motorTimer);
      }
    }
  }
}
