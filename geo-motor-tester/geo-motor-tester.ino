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
#define Motor1BrakePin 20
#define Motor2DirectionPin 23
#define Motor2BrakePin 22

void setup()
{
  SPI.begin();

  dac.outputA(0); // 0 - 4095
  dac.outputB(0);
  dac.latch();

  pinMode(Motor1DirectionPin, OUTPUT); // output is reverse; input is forward
  digitalWrite(Motor1DirectionPin, LOW);
  pinMode(Motor2DirectionPin, INPUT);
  digitalWrite(Motor2DirectionPin, LOW);

  pinMode(Motor1BrakePin, INPUT);
  pinMode(Motor2BrakePin, INPUT);

  Serial.begin(115200);
}

void loop()
{
  unsigned long dv = 800; // starting value

  unsigned long cap_r = 1100;
  unsigned long cap_l = 1220;

/* left 1120 - 1830
   right 

*/

  while (1) {
    dac.outputA(dv > cap_r ? cap_r:dv); // right
    dac.outputB(dv > cap_l ? cap_l:dv); // left
    dac.latch();

    Serial.println(dv);
    delay(500);
    dv = dv + 10;
    if (dv >= 4096) {
      dv = 0;
    }
  }
}
