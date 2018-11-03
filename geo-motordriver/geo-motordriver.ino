#include <Arduino.h>
#include <SPI.h>
#include <DAC_MCP49xx.h>
#include <PID_v1.h>
#include "Movement.h"

//#define DEBUG

#define MCP4921_LDAC_PIN 3
#define MCP4921_SS_PIN 10

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

#define mcpShutdown 2

unsigned long motorTimer = 0;
unsigned long brakeTimer = 0;

#define kFORWARD 1
#define kBACKWARD -1
#define kZERO 0

// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200

// how many bits we lop off the number of microseconds for a given
// pulse measurement. We don't need all the accuracy (and our accuracy is
// skewed by the time spent in code anyway). 
#define bitShiftModifier 7

// left maxes out ~33000
#define LmaxTime 63000
// right maxes out ~83000
#define RmaxTime 100000

typedef struct _sensorState {
  bool a;
  bool b;
  bool c;
} sensorState;

typedef struct _sensorReading {
  bool isValid;
  bool movingForward;
  bool movingBackward;
  unsigned long periodA;
  unsigned long periodB;
  unsigned long periodC;
  unsigned long estimatedPeriodOverall;
} sensorReading;

typedef struct _sensorHistory {
  unsigned long startA;
  unsigned long periodA;
  unsigned long startB;
  unsigned long periodB;
  unsigned long startC;
  unsigned long periodC;

  unsigned long timeOfLastReading;
} sensorHistory;

// convenience macros for determining which sensors are "on"
// (low). Note repitition of logic for ease of understanding (e.g. AB
// is the same as BA).
#define isA(x) ((x).a==LOW && (x).b==HIGH && (x).c==HIGH)
#define isAB(x) ((x).a==LOW && (x).b==LOW && (x).c==HIGH)
#define isB(x) ((x).a==HIGH && (x).b==LOW && (x).c==HIGH)
#define isBC(x) ((x).a==HIGH && (x).b==LOW && (x).c==LOW)
#define isC(x) ((x).a==HIGH && (x).b==HIGH && (x).c==LOW)
#define isCA(x) ((x).a==LOW && (x).b==HIGH && (x).c==LOW)

#define isBA(x) ((x).a==LOW && (x).b==LOW && (x).c==HIGH)
#define isCB(x) ((x).a==HIGH && (x).b==LOW && (x).c==LOW)
#define isAC(x) ((x).a==LOW && (x).b==HIGH && (x).c==LOW)

#define isABC(x) ((x).a==LOW && (x).b==LOW && (x).c==LOW)
#define isNONE(x) ((x).a==HIGH && (x).b==HIGH && (x).c==HIGH)

void readSensors(sensorState *s, bool isLeft)
{
  bool a = true, b=true, c=true;

  //  unsigned long startTime = millis();

  // coded for rollover...
  //  while (startTime + 20 > millis()) {
  if (digitalRead(isLeft ? LsensorA : RsensorA) == LOW) {
    a = false;
  }
  if (digitalRead(isLeft ? LsensorB : RsensorB) == LOW) {
    b = false;
  }
  if (digitalRead(isLeft ? LsensorC : RsensorC) == LOW) {
    c = false;
  }
  //  }

  s->a = a;
  s->b = b;
  s->c = c;
}

void measureSensor(unsigned long sampleTime,
		   sensorState *currentState, sensorState *previousState,
		   sensorHistory *history, sensorReading *currentReading,
		   sensorReading *previousReading, unsigned long maxTime) {

  // To get speed, we need to measure the off-pulse length. If we have
  // *none*, then we'll copy anything that exists in the old one.

  // Reset the current reading
  currentReading->periodA = currentReading->periodB = currentReading->periodC = 0;

  if (currentState->a == false) {
    if (history->startA == 0) {
      history->startA = sampleTime;
      // can't tell the speed yet; we just started                          
    } else {
      // We know the period now, b/c we know when it started.               
      history->periodA = sampleTime - history->startA;
      history->timeOfLastReading = sampleTime;
      // ... but we might be wrong b/c of some transient noise. So          
      // we don't reset rightSensor.startA. If the next read is             
      // also low, we'll extend rightSensor.periodA.                        
    }
  } else {
    // ... an all-high period tells us to reset rightSensor.startA.           
    history->startA = 0;

    if (history->periodA) {
      // This is a true reading now.                                        
      currentReading->periodA = history->periodA;
      history->periodA = 0;
    }
  }

  if (currentState->b == false) {
    if (history->startB == 0) {
      history->startB = sampleTime;
    } else {
      history->periodB = sampleTime - history->startB;
      history->timeOfLastReading = sampleTime;
    }
  } else {
    history->startB = 0;
    if (history->periodB) {
      currentReading->periodB = history->periodB;
    }
    history->periodB = 0;
  }

  if (currentState->c == false) {
    if (history->startC == 0) {
      history->startC = sampleTime;
    } else {
      history->periodC = sampleTime - history->startC;
      history->timeOfLastReading = sampleTime;
    }
  } else {
    history->startC = 0;
    if (history->periodC) {
      currentReading->periodC = history->periodC;
    }
    history->periodC = 0;
  }
  if (currentReading->periodA) {
    currentReading->isValid = true;
    currentReading->estimatedPeriodOverall = currentReading->periodA >> bitShiftModifier;
  } else if (currentReading->periodB) {
    currentReading->isValid = true;
    currentReading->estimatedPeriodOverall = currentReading->periodB >> bitShiftModifier;
  } else if (currentReading->periodC) {
    currentReading->isValid = true;
    currentReading->estimatedPeriodOverall = currentReading->periodC >> bitShiftModifier;
  } else if ((sampleTime - history->timeOfLastReading) >= 100000) {
    currentReading->isValid = true;
    currentReading->movingForward = currentReading->movingBackward = false;
    currentReading->estimatedPeriodOverall = 0;
  }

  if (!currentReading->periodA && !currentReading->periodB && !currentReading->periodC) {
    currentReading->periodA = previousReading->periodA;
    currentReading->periodB = previousReading->periodB;
    currentReading->periodC = previousReading->periodC;
  }

  // To find direction, we need to know the firing order of A, B, C.
  // Either we're going AB B BC C CA A or we're going AC C CB B BA A.
  // Start by assuming we're in the exact state as the previous time,
  // and then check to see if anything has changed.  We look two steps
  // in the future in case we missed one.
  currentReading->movingForward = previousReading->movingForward;
  currentReading->movingBackward = currentReading->movingBackward;
  if (isAB(*previousState)) {
    // forward would be AB -> B; backward would be BA -> A.  FIXME: Do
    // we need double-jump testing (AB -> BC and BA -> AC)?
    if (isB(*currentState) || isBC(*currentState)) {
      currentReading->movingForward = true;
      currentReading->movingBackward = false;
    }
    else if (isA(*currentState) || isAC(*currentState)) {
      currentReading->movingBackward = true;
      currentReading->movingForward = false;
    }

  } else if (isB(*previousState)) {
    // Forward would be B -> BC; backward would be B -> BA.
    if (isBC(*currentState) || isC(*currentState)) {
      currentReading->movingForward = true;
      currentReading->movingBackward = false;
    } else if (isBA(*currentState) || isA(*currentState)) {
      currentReading->movingBackward = true;
      currentReading->movingForward = false;
    }

  } else if (isBC(*previousState) || isC(*previousState)) {
    // Forward would be BC -> C; backward would be BC -> B.
    if (isC(*currentState)) {
      currentReading->movingForward = true;
      currentReading->movingBackward = false;
    } else if (isB(*currentState) || isBA(*currentState)) {
      currentReading->movingBackward = true;
      currentReading->movingForward = false;
    }
  } else if (isC(*previousState)) {
    // Forward would be C -> CA; backward would be C -> CB.
    if (isCA(*currentState) || isA(*currentState)) {
      currentReading->movingForward = true;
      currentReading->movingBackward = false;
    } else if (isCB(*currentState) || isB(*currentState)) {
      currentReading->movingBackward = true;
      currentReading->movingForward = false;
    }
  } else if (isCA(*previousState)) {
    // Forward would be CA -> A; backward would be CA -> C.
    if (isA(*currentState) || isAB(*currentState)) {
      currentReading->movingForward = true;
      currentReading->movingBackward = false;
    } else if (isC(*currentState) || isCB(*currentState)) {
      currentReading->movingBackward = true;
      currentReading->movingForward = false;
    }
  } else if (isA(*previousState)) {
    // Forward would be A -> AB; backward would be A -> AC.
    if (isAB(*currentState) || isB(*currentState)) {
      currentReading->movingForward = true;
      currentReading->movingBackward = false;
    } else if (isAC(*currentState) || isC(*currentState)) {
      currentReading->movingBackward = true;
      currentReading->movingForward = false;
    }
  }
  if (currentReading->estimatedPeriodOverall >= maxTime) {
    // We're stopped.
    currentReading->movingForward = currentReading->movingBackward = 0;
    currentReading->estimatedPeriodOverall = 0;
  }

}


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
  pinMode(Motor1BrakePin, INPUT);
  pinMode(Motor2DirectionPin, INPUT);
  pinMode(Motor2BrakePin, INPUT);

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
  static bool lastLeftBrake = false;
  static bool lastRightBrake = false;

  if (leftBrake) {
    if (lastLeftBrake != isOn) {
      if (isOn) { // active-low; drag to ground
	pinMode(Motor2BrakePin, OUTPUT);
	digitalWrite(Motor2BrakePin, LOW);
      } else { // float high to disable brake
	pinMode(Motor2BrakePin, INPUT);
      }
      lastLeftBrake = isOn;
    }
  } else {
    if (lastRightBrake != isOn) {
      if (isOn) { // active-low; drag to ground
	pinMode(Motor1BrakePin, OUTPUT);
	digitalWrite(Motor1BrakePin, LOW);
      } else { // float high to disable brake
	pinMode(Motor1BrakePin, INPUT);
      }
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
  static sensorState currentLeftState = { true, true, true }, 
    previousLeftState = { true, true, true };
  static sensorReading currentLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorHistory leftSensor = { 0, 0, 0, 0, 0, 0, 0 };

  static sensorState currentRightState = { true, true, true }, 
    previousRightState = { true, true, true };
  static sensorReading currentRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorHistory rightSensor = { 0, 0, 0, 0, 0, 0, 0 };

  unsigned long sampleTime = micros();
  readSensors(&currentLeftState, 0);
  readSensors(&currentRightState, 1);

  // Interpret those readings
  measureSensor(sampleTime, &currentLeftState, &previousLeftState, &leftSensor, &currentLeftReading, &previousLeftReading, LmaxTime);
  measureSensor(sampleTime, &currentRightState, &previousRightState, &rightSensor, &currentRightReading, &previousRightReading, RmaxTime);

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
