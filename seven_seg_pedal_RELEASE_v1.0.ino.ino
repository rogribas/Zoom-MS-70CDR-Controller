#include <Wire.h>
#include "SevenSeg.h"

byte lA = 4;
byte lB = 6;
byte lC = 10;
byte lD = 8;
byte lE = 9;
byte lF = 12;
byte lG = 7;

SevenSeg disp (lA, lB, lC, lD, lE, lF, lG);
const int numOfDigits = 2;
int digitPins[numOfDigits] = {5, 11};
static int brightness = 10;

byte preset = 0;

byte const ERROR_NUM = 100;

bool initConnection = true;

byte listLeds[] = {
  lA, lF, lG, lC, lD, lE,
  lG, lB, lA, lF, lE, lD,
  lD, lC, lG, lF, lA, lB,
  lG, lE, lD, lC, lB, lA,
};

void setup() {
  disp.setDigitPins(numOfDigits, digitPins);

  disp.setDutyCycle(10);

  Serial.begin(115200);
  Serial.println("initConnection");
  
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
}

void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.println(c);         // print the character
  }
  preset = Wire.read();    // receive byte as an integer
  Serial.println(preset);         // print the character 
  initConnection = false;
}

void loop() {
  if (preset == ERROR_NUM) {
    disp.write("Er");
  }
  else if (initConnection) {
    initDisplay();
  }
  else {
    disp.write(preset);
  }
}

void initDisplay() {
  byte listLedsLength = sizeof(listLeds);
  for (byte iNow = 0; iNow < listLedsLength; ++iNow) {
    byte iPast1 = (iNow + listLedsLength - 1) % listLedsLength;
    byte iPast2 = (iNow + listLedsLength - 2) % listLedsLength;
    byte iPast3 = (iNow + listLedsLength - 3) % listLedsLength;

    for (int k = 0; k < 200; k++) {
      if (iNow < 12) disp.changeDigit(0);
      else disp.changeDigit(1);
      digitalWrite(listLeds[iNow], LOW);

      if (iPast1 < 12) disp.changeDigit(0);
      else disp.changeDigit(1);
      digitalWrite(listLeds[iPast1], LOW);

      if (iPast2 < 12) disp.changeDigit(0);
      else disp.changeDigit(1);
      digitalWrite(listLeds[iPast2], LOW);
  
      if (iPast3 < 12) disp.changeDigit(0);
      else disp.changeDigit(1);
      digitalWrite(listLeds[iPast3], HIGH);
    }
  }  
}



