#include <Arduino.h>
#include <SPI.h>
#include <DAC_MCP49xx.h>

/*
Right motor (DAC A), forward (pin 23 INPUT): 1110 - 2970
  reverse (pin 23 OUTPUT): 1110 - 2020
Left motor (DAC B), forward (pin 21 INPUT): 1290 - 3030
  reverse (pin 21 OUTPUT): 1280 - 2090

  it will not letyou jump to a fast backwards speed
*/
#define MCP4921_LDAC_PIN 3
#define MCP4921_SS_PIN 10

DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, MCP4921_SS_PIN, MCP4921_LDAC_PIN);

#define Motor1DirectionPin 21
#define Motor2DirectionPin 23

void setup()
{
  SPI.begin();

  dac.outputA(0); // 0 - 4095
  dac.outputB(0);
  dac.latch();

  pinMode(Motor1DirectionPin, OUTPUT); // output is reverse; input is forward
  pinMode(Motor2DirectionPin, OUTPUT);

  Serial.begin(115200);
}

void loop()
{
  unsigned long dv = 1000;

  /* Reliable values?
     
     each of these is starts-at / reliable-value

              l            r
     step 0:  ---------    1090/1110
     step 1:  1300/1310    1120/1140
     step 2:  1320/1350    1160/1180
     step 3:  1370/1390    1200/1220
   */

  unsigned long cap_r = 1220; // 1090 1130 1160 1200 1230 1270 1300 1340 1370 
  unsigned long cap_l = 1350; // 1300 1320 1370 1410 1440 1480 1520 1550 1590

  while (1) {
    dac.outputA(dv > cap_r ? cap_r:dv); // right
    dac.outputB(dv > cap_l ? cap_l:dv); // left
    dac.latch();

    Serial.println(dv);
    delay(1000);
    dv = dv + 10;
    if (dv >= 4096) {
      dv = 0;
    }
  }
}
