#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <RingBuffer.h>
#include <SoftwareSerial.h>

//#define DEBUG

/* Moteino constants */
#define NODEID      32
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
RingBuffer recvBuffer(32);

#define SSERRX 9 // Not used; doubled up with onboard LED
#define SSERTX A0
SoftwareSerial softSerial(SSERRX, SSERTX);

void MakeMotorsGoTank(int8_t leftJoystickY, int8_t rightJoystickY)
{
  // debugging: pulse the LED when we get a movement command
  static bool ledState = 0;
  ledState = !ledState;
  digitalWrite(9, ledState);

}

void setup() {
  // put your setup code here, to run once:
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  flash.initialize();

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Hello from Geo's receiver");
#endif
  softSerial.begin(57600);

}

void loop() {
  if (radio.receiveDone()) {
    // Put all the radio data in a buffer quickly in case more data comes in while we're processing it
    recvBuffer.addBytes(radio.DATA, radio.DATALEN);
  }

  while (recvBuffer.hasData()) {
    unsigned char d = recvBuffer.consumeByte();
    
      switch (d) {
        case '*':
          // Restart!
          //ForceRestart();
          break;
          
	  /* Have a (tank tread) joystick position: set motor targets */
      case 'T':
	if (recvBuffer.count() >= 2) {
	  softSerial.write('T');
	  uint8_t c = recvBuffer.consumeByte();
	  uint8_t c2 = recvBuffer.consumeByte();
#ifdef DEBUG
	  Serial.print("T ");
	  Serial.print(c);
	  Serial.print(" ");
	  Serial.println(c2);
#endif
	  softSerial.write(c);
	  softSerial.write(c2);
	}
	break;

        /* Slow/fast mode */
        case '+':
	  softSerial.write('+');
          break;
        case '-':
	  softSerial.write('-');
          break;

	  /* brake */
      case 'm':
	softSerial.write('m');
	break;
  
      }
  }
}
