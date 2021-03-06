#include <stdio.h>

#ifdef _UNIX
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
#else
#include <Arduino.h>
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)

#define kLEFT 0
#define kRIGHT 1

class Movement {
 public:
  Movement() { actual_l = actual_r = target_l_pct = target_r_pct = 0; brakeEnabled = false; decelLeft = decelRight = false; stopped_l_time = stopped_r_time = 0; fastMode = false; };
  ~Movement() {};

  void Update();
  void JoystickTankTarget(float x, float y);
  void JoystickPolarTarget(float x, float y);
  void GetMotorTargetPercentages(int8_t *left, int8_t *right);
  void GetMotorTargetValues(int16_t *left, int16_t *right);
  void GetMotorValues(int16_t *left, int16_t *right);
  void GetMotorDirection(int8_t *left, int8_t *right);
  float GetMotorRatio();
  void Brake(bool set);
  void GetBrakeState(bool *left, bool *right);

  void Reset(); // called if there's an error in the engine...

  // difference between the two states (skipping the dead zone)
  int16_t MotorDifferenceMagnitude(int16_t a, int16_t b, int8_t whichMotor);

  void setFastMode(bool isFast);

 protected:
  int16_t percentToMotorDriverValue(int8_t pct, bool isTurning, int8_t whichMotor);
  int16_t performAccelerationWithConstraints(int16_t current,
					     int16_t target,
					     int16_t maxVal,
					     int8_t accel,
					     bool *isDecelOut,
					     int8_t whichMotor);

 private:
  int8_t target_l_pct, target_r_pct; // percentages, -100..100
  int16_t actual_l, actual_r; // motor driver values, -MAXMOTOR..MAXMOTOR
  int16_t stopped_l_time, stopped_r_time;
  bool brakeEnabled;
  bool decelLeft, decelRight; // whether or not we're intentionally decelerating each motor
  bool fastMode;
};
