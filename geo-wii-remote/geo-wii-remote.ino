#include <Arduino.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69
#include <RingBuffer.h>    //get it here: https://github.com/JorjBauer/RingBuffer
#include <Wire.h>
#include "WiiClassic.h"

/* Pins used.
 *  
 *  The Moteino communicates with its embedded RFM69 on pins 2, 10, 11, 12, 13.
 *  Pin 9 is the onboard LED, but it's not used.
 *  Pin 6 is used as a PWM for the (top) LED. (The front LED is
 *   connected to the charger.)
 *  Pin A6 and A7 are analog input only.
 *  
 *   0
 *   1
 *   2 RFM69: INT0
 *   3 
 *   4
 *   5
 *   6 LED
 *   7
 *   8
 *   9 (onboard LED; reusable if necessary)
 *  10 RFM69
 *  11 RFM69
 *  12 RFM69
 *  13 RFM69
 *  14/A0 
 *  15/A1 
 *  16/A2
 *  17/A3
 *  18/A4 Wire library I2C
 *  19/A5 Wire library I2C
 */

/* Moteino constants */
#define DALEK_NODEID 32 // Our partner in crime - Geo's ID
#define NODEID      33
#define NETWORKID   212
#define FREQUENCY RF69_915MHZ
//#define IS_RFM69HW  //uncomment only for RFM69HW! Leave out if you have RFM69W!
#ifdef DEFAULTKEY
#define ENCRYPTKEY DEFAULTKEY
#else
#pragma message("Default encryption key not found; using compiled-in default instead")
#define ENCRYPTKEY "sampleEncryptKey"
#endif
#define FLASH_SS 8

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); // 0xEF30 is windbond 4mbit

#define LED_PIN 6
#define LED_STEP 1
#define LED_BRIGHTNESS 8
#define LED_DELAY 100

WiiClassic ctrl = WiiClassic();
int lcx, lcy, rcx, rcy;
// How wide is each half of the left joystick's "center/NULL" range?
#define LEFT_CENTER_RANGE 8
// Same with the right joystick (which has a different precision)
#define RIGHT_CENTER_RANGE 4

#define kFORWARD 0
#define kBACKWARD 1

//#define DEBUG

unsigned long last_data_update = 0;

/* LED pulsing variables */
int led_brightness = LED_BRIGHTNESS;
int led_direction = - (LED_STEP);
unsigned long led_timer = 0;

/* Movement state - used to tell whether or not to keep sending data */
bool wasMoving = false;

/* Radio transmission buffer */
#define RADIOBUFSIZE 16
RingBuffer radioBuffer(RADIOBUFSIZE);

void setup() {
#ifdef DEBUG
  Serial.begin(57600);
  Serial.println("init");
#endif

  Wire.begin();
//  nunchuck_setpowerpins();
  ctrl.begin();
  ctrl.update();

  // Read the center positions of the joysticks on start-up
  lcx = ctrl.leftStickX();
  lcy = ctrl.leftStickY();
  rcx = ctrl.rightStickX();
  rcy = ctrl.rightStickY();

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  if (!flash.initialize()) {
#ifdef DEBUG
  Serial.println("Failed to init flash");
#endif
  }

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, LED_BRIGHTNESS);

#ifdef DEBUG  
  Serial.print("Startup centering: left ");
  Serial.print(lcx);
  Serial.print(",");
  Serial.print(lcy);
  Serial.print("; right ");
  Serial.print(rcx);
  Serial.print(",");
  Serial.println(rcy);
#endif
}

// "in" from 0-100, linear; translated to nonlinear 0-100 that favors           
// the lower end                                                                
int trans(int in)
{
  return in;

  // The base 2/3 should be 20% of the output
  if (in <= 66) {
    return map(in, 0, 66, 0, 20);
  }

  // up to 90% of the range is 20% - 80% of the output range
  if (in <= 90) {
    return map(in, 66, 90, 20, 80);
  }

  // And the remainder                                                          
  return map(in, 90, 100, 80, 100);
}

int HandleSlowFastMode()
{
  if (ctrl.selectPressed()) { // "select" is also "-"
    rf_send('-');
    return 1;
  } else if (ctrl.startPressed()) { // "start" is also "+"
    rf_send('+');
    return 1;
  }
  return 0; // nothing sent to remote end.
}

// return x/y values that are floats [-100..100]
// The left joystick gives 0..40 for down/left and 41..63 for up/right.
uint8_t ReadLeftJoystick(float *x, float *y)
{
  int lrAnalog = ctrl.leftStickX();
  int udAnalog = ctrl.leftStickY();

  *x = *y = 0;

  // This is the left joystick, so its range is [0,63]. We have a center 
  // position that we cached on startup, which is [lcx, lcy] and we know 
  // the range of the "dead" middle spot we want, which is LEFT_CENTER_RANGE.
  //
  // Turn all of this in to a percentage of left (negative) or right (positive)
  // and up (positive) or down (negative), from -1.0 to 1.0.

  // If the joystick is dead center, then it's nothing.
  if (lrAnalog >= (lcx - LEFT_CENTER_RANGE) &&
      lrAnalog <= (lcx + LEFT_CENTER_RANGE) &&
      udAnalog >= (lcy - LEFT_CENTER_RANGE) &&
      udAnalog <= (lcy + LEFT_CENTER_RANGE)) {
    // Dead center; do nothing.
    return 0;
  }

  if (lrAnalog < lcx - LEFT_CENTER_RANGE) {
    // Left-leaning
    float range = (lcx - LEFT_CENTER_RANGE);
    *x = -(1.0 - ((float)lrAnalog / range));
  } else if (lrAnalog > lcx + LEFT_CENTER_RANGE) {
    // Right-leaning
    float range = 63.0 - (lcx + LEFT_CENTER_RANGE);
    *x = (((float)lrAnalog - ((float)lcx + (float)LEFT_CENTER_RANGE) ) / range);
  }

  if (udAnalog < lcy - LEFT_CENTER_RANGE) {
    // down-leaning
    float range = (lcy - LEFT_CENTER_RANGE);
    *y =  -(1.0 - ((float)udAnalog / range));
 
  } else if (udAnalog > lcy + LEFT_CENTER_RANGE) {
    float range = 63.0 - (lcy + LEFT_CENTER_RANGE);
    *y = (float)(udAnalog - lcy - LEFT_CENTER_RANGE) / range;
  }

  // Translate: make the center region of the joystick "bigger" and the outer ring "smaller" so that it's easier to go slowly
  int tmp = (int)abs(((float)(*x) * (float)100.0)); // make it 0..100
  tmp = trans(tmp); // perform non-linear adaptation
  // preserve original sign
  if (*x > 0) {
    *x = (float)tmp;
  } else {
    *x = -(float)tmp;
  }
  
  tmp = (int)abs(((float)(*y) * (float)100.0));
  tmp = trans(tmp);
  if (*y >= 0) {
    *y = (float)tmp;
  } else {
    *y = -(float)tmp;
  }

#ifdef DEBUG
  Serial.print("l=");
  Serial.print(udAnalog);
  Serial.print(" ");
  Serial.println(*y);
#endif
  return 1;
}

// return x/y values that are floats [-100..100]
uint8_t ReadRightJoystick(float *x, float *y)
{
  int lrAnalog = ctrl.rightStickX();
  int udAnalog = ctrl.rightStickY();

  *x = *y = 0;

  // This is the right joystick, so its range is [0,31]. We have a center 
  // position that we cached on startup, which is [rcx, rcy] and we know 
  // the range of the "dead" middle spot we want, which is RIGHT_CENTER_RANGE.
  //
  // Turn all of this in to a percentage of left (negative) or right (positive)
  // and up (positive) or down (negative), from -1.0 to 1.0.

  // If the joystick is dead center, then it's nothing.
  if (lrAnalog >= (rcx - RIGHT_CENTER_RANGE) &&
      lrAnalog <= (rcx + RIGHT_CENTER_RANGE) &&
      udAnalog >= (rcy - RIGHT_CENTER_RANGE) &&
      udAnalog <= (rcy + RIGHT_CENTER_RANGE)) {
    // Dead center; do nothing.
    return 0;
  }

  if (lrAnalog < rcx - RIGHT_CENTER_RANGE) {
    // Left-leaning
    float range = (rcx - RIGHT_CENTER_RANGE);
    *x = -(1.0 - ((float)lrAnalog / range));
  } else if (lrAnalog > rcx + RIGHT_CENTER_RANGE) {
    // Right-leaning
    float range = 31.0 - (rcx + RIGHT_CENTER_RANGE);
    *x = (((float)lrAnalog - ((float)rcx + (float)RIGHT_CENTER_RANGE) ) / range);
  }

  if (udAnalog < rcy - RIGHT_CENTER_RANGE) {
    // down-leaning
    float range = (rcy - RIGHT_CENTER_RANGE);
    *y =  -(1.0 - ((float)udAnalog / range));
 
  } else if (udAnalog > rcy + RIGHT_CENTER_RANGE) {
    float range = 31.0 - (rcy + RIGHT_CENTER_RANGE);
    *y = (float)(udAnalog - rcy - RIGHT_CENTER_RANGE) / range;
  }

  // Translate: make the center region of the joystick "bigger" and the outer ring "smaller" so that it's easier to go slowly
  int tmp = (int)abs(((float)(*x) * (float)100.0)); // make it 0..100
  tmp = trans(tmp); // perform non-linear adaptation
  // preserve original sign
  if (*x > 0) {
    *x = (float)tmp;
  } else {
    *x = -(float)tmp;
  }
  
  tmp = (int)abs(((float)(*y) * (float)100.0));
  tmp = trans(tmp);
  if (*y >= 0) {
    *y = (float)tmp;
  } else {
    *y = -(float)tmp;
  }

#ifdef DEBUG
  Serial.print("r=");
  Serial.print(udAnalog);
  Serial.print(" ");
  Serial.println(*y);
#endif
  return 1;
}

int HandleMovement()
{
  int ret = 0;

  float rx, ry, lx, ly;
  ReadRightJoystick(&rx, &ry);
  ReadLeftJoystick(&lx, &ly);

  // If we are moving now (x or y are not zero), or if we *were* moving 
  // the last time we were called, then send an update.
  if (wasMoving || ly != 0 || ry != 0) {
    char gobuf[3] = { 'T', (char)ly, (char)ry };
    rf_send(gobuf, 3);
    ret = 1;
  }

  // Then note whether or not we were moving this time so we know what to 
  // do next time.
  wasMoving = (ly != 0 || ry != 0);

  return ret;
}

int HandleBrakes()
{
  int ret = 0;
  if (ctrl.leftShoulderPressed() || ctrl.leftShouldPressure()) {
    // brakes
    rf_send("m", 1);
    ret = 1;
  } else if (ctrl.rightShoulderPressed() || ctrl.rightShouldPressure()) {
    // brakes
    rf_send("m", 1);
    ret = 1;
  }
  return ret;
}

void rf_send(const char *data, int datasize)
{
#ifdef DEBUG
  //  Serial.println(data);
#endif
  // Queue data for later transmission, so we're sending it all at once.
  radioBuffer.addBytes(data, datasize);
}

void rf_send(const char data)
{
#ifdef DEBUG
  //  Serial.println(data);
#endif
  rf_send(&data, 1);
}

bool rf_sendQueue()
{
  // if we have any data queued for sending, send it now.
  if (radioBuffer.hasData()) {
    unsigned char data[RADIOBUFSIZE];
    int datasize = radioBuffer.count();
    for (int i=0; i<datasize; i++) {
      data[i] = radioBuffer.consumeByte();
    }
    radio.send(DALEK_NODEID, data, datasize, false); // no ACK, no retransmission.
    return true;
  }
  return false;
}

void loop() {
  // Check for program updates over RF
  if (radio.receiveDone()) {
    CheckForWirelessHEX(radio, flash, true); // checks for the header 'FLX?' and reflashes new program if it finds one
  }
  
  // Handle LED pulsing
  if (millis() >= led_timer) {
    led_brightness += led_direction;
    if (led_brightness <= 0 || led_brightness >= LED_BRIGHTNESS) {
      if (led_brightness < 0)
	led_brightness = 0;
      if (led_brightness > LED_BRIGHTNESS)
	led_brightness = LED_BRIGHTNESS;

      led_direction = -led_direction;
    }
    analogWrite(LED_PIN, led_brightness);
    led_timer = millis() + LED_DELAY;
  }

  // Periodically update the data
  if (millis() >= last_data_update + 80) {
    // Read new values from the controller
    ctrl.update();

    int didSend = HandleBrakes();
    if (!didSend) {
      // don't try to move if the brakes are on.
      didSend |= HandleMovement();
    }
    didSend |= HandleSlowFastMode();

    rf_sendQueue();

    if (didSend) {
      led_brightness = LED_BRIGHTNESS;
      led_direction = - (LED_STEP);
    }
    last_data_update = millis();
  }

}
