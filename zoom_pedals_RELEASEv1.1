#include <Wire.h>
#include <Usb.h>
#include <usbhub.h>
#include <usbh_midi.h>
#include <SoftPWM.h>

#define DEBUG 1

USB Usb;
USBH_MIDI Midi(&Usb);


//Input pin declaration
const byte inPinUP = 3;
const byte inPinDOWN = 4;
const byte inPinPEDAL1 = 6;
const byte inPinPEDAL2 = 5;
const byte inPinPEDAL3 = 2;

// Output pin declaration
const byte outPin1 = 15; // Led Pedal 1
const byte outPin2 = 16; // Led Pedal 2
const byte outPin3 = 17; // Led Pedal 3
const byte outPinErr = 14; // Led INFO

const byte ERROR_NUM = 100;

//Current reading from the input pins
byte readingUP;
byte readingDOWN;
byte readingPEDAL1;
byte readingPEDAL2;
byte readingPEDAL3;

// Previous reading from the input pins
byte previousUP = LOW;
byte previousDOWN = LOW;
byte previousPEDAL1 = LOW;
byte previousPEDAL2 = LOW;
byte previousPEDAL3 = LOW;

// DEBOUNCE SEQUENCE ---
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 350;   // the debounce time, increase if the output flickers

// PEDAL STATUS VARIABLES
byte pedalStatus1 = 0;
byte pedalStatus2 = 0;
byte pedalStatus3 = 0;

#define SYSEX_MSG_LENGTH 160
uint8_t sysExMsg[SYSEX_MSG_LENGTH];
uint8_t indexSysExMsg;
byte currentPreset = 0;

void setup()
{
  pinMode(inPinUP, INPUT);
  pinMode(inPinDOWN, INPUT);
  pinMode(outPin1, OUTPUT);
  pinMode(outPin2, OUTPUT);
  pinMode(outPin3, OUTPUT);
  pinMode(outPinErr, OUTPUT);

  pinMode(7, OUTPUT);  // For the sheild
  digitalWrite(7, HIGH); // for the sheild
  
  Wire.begin(); // join i2c bus (address optional for master)

  if (Usb.Init() == -1)
  {
    digitalWrite(outPinErr, HIGH);
    while(1);               // Halt
  }

  loadingLeds(4800);

  Serial.begin(115200);
  Serial.println("INIT");  

  SendEditorMode(); loadingLeds(300);
  SendEditorMode(); loadingLeds(300);
  SendEditorMode(); loadingLeds(300);
  SendEditorMode(); loadingLeds(300);
  bool communicationOK = SendEditorMode();

  if (!communicationOK) errorIndication();

  // Update program number display
  currentPreset = SendSysexGetProgram();
  updateLedsNumber(currentPreset + 1);
  // Update pedals
  SendSysexUpdatePedals();
}

void loop()
{
  readingUP = digitalRead(inPinUP);
  readingDOWN = digitalRead(inPinDOWN);
  readingPEDAL1 = digitalRead(inPinPEDAL1);
  readingPEDAL2 = digitalRead(inPinPEDAL2);
  readingPEDAL3 = digitalRead(inPinPEDAL3);

  if (millis() - time > debounce) {
    if (readingUP == HIGH) {
      // Mirar on estem
      // byte currentPreset = SendSysexGetProgram();
      // Serial.print("current preset = ");
      // Serial.println(currentPreset);
      
      // Moure un cap amunt (+1)
      currentPreset = (currentPreset + 1) % 50;
      SendChangeProgram(currentPreset);
      updateLedsNumber(currentPreset+1);
      previousUP = HIGH;
          
      time = millis();
    }
    else if (readingDOWN == HIGH) {
      // Mirar on estem
      // byte currentPreset = SendSysexGetProgram();
      // Serial.print("current preset = ");
      // Serial.println(currentPreset);
      
      // Moure un cap avall (-1)
      currentPreset = (currentPreset + 50 - 1) % 50;
      SendChangeProgram(currentPreset);
      updateLedsNumber(currentPreset+1);
      previousDOWN = HIGH;
      
      time = millis();
    }
    else if (previousUP == HIGH && readingUP == LOW) {
      SendSysexUpdatePedals();
      previousUP = LOW;
    }
    else if (previousDOWN == HIGH && readingDOWN == LOW) {
      SendSysexUpdatePedals();
      previousDOWN = LOW;
    }
    
    // PEDALS
    else if (readingPEDAL1 == HIGH && previousPEDAL1 == LOW) {
      togglePedal(0);
      previousPEDAL1 = HIGH;
    }
    else if (readingPEDAL2 == HIGH && previousPEDAL2 == LOW) {
      togglePedal(1);
      previousPEDAL2 = HIGH;
    }
    else if (readingPEDAL3 == HIGH && previousPEDAL3 == LOW) {
      togglePedal(2);
      previousPEDAL3 = HIGH;
    }
    else if (readingPEDAL1 == LOW && previousPEDAL1 == HIGH) {
      previousPEDAL1 = LOW;
    }
    else if (readingPEDAL2 == LOW && previousPEDAL2 == HIGH) {
      previousPEDAL2 = LOW;
    }
    else if (readingPEDAL3 == LOW && previousPEDAL3 == HIGH) {
      previousPEDAL3 = LOW;
    }
  }

  Usb.Task();
  uint32_t t1 = (uint32_t)micros();
  if ( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    MIDI_poll();
  }

  doDelay(t1, (uint32_t)micros(), 1000);
}

// Send "Program Change" MIDI Message
void SendChangeProgram(byte number)
{
  Usb.Task();
  if (Usb.getUsbTaskState() == USB_STATE_RUNNING)
  {
    Serial.println(">>>> Serial MIDI OK!!!");
    byte Message[2];                 // Construct the midi message (2 bytes)
    Message[0]=0xC0;                 // 0xC0 is for Program Change 
    Message[1]=number;               // Number is the program/patch 
    Midi.SendData(Message);          // Send the message
    delay(10);
  }
  else
  { 
    Serial.println(">>>> Serial MIDI F A I L !!!");
  }
}

boolean SendEditorMode() {
  //F0 52 00 61 50 F7 
  Usb.Task();
  Serial.println(Usb.getUsbTaskState() == USB_STATE_RUNNING);
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    uint8_t sysexData[] = { 0xF0, 0x52, 0x00, 0x61, 0x50, 0xF7 };
    Midi.SendSysEx(sysexData, sizeof(sysexData));
    delay(10);
    Serial.println("COMMUNICATION READY");
    return true;
  }
  else
  {
    Serial.println("ERROR");
    return false;   
  }
  
}

byte SendSysexGetProgram()
{
  byte currentPreset = 100;
  
  Usb.Task();
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    uint8_t outBuf[8];
    uint8_t sizeData;

    while(currentPreset == 100) {
      Serial.println(" - - > READ CURRENT PRESET!");
      // F0 52 00 61 33 F7
      uint8_t sysexData[] = { 0xF0, 0x52, 0x00, 0x61, 0x33, 0xF7 };
      uint8_t scode = Midi.SendSysEx(sysexData, sizeof(sysexData));
  
      
      // READ
      Serial.println(" read START");

      byte counter = 0;
      while(currentPreset == 100 && counter < 50) {
        currentPreset = MIDI_poll_getCurrentPreset();
        Serial.println(" --> current preset = ");
        Serial.println(currentPreset);
        delay(50);
        counter++;
      }
    }
    Serial.println(" read END");
  }
  else { Serial.println(" - - > SERIAL NOT WORKINGG!"); }
  return currentPreset;
}

void togglePedal(byte pedalID) {
  Usb.Task();
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    Serial.println(" - - > TOGGLE PEDAL!");
    byte nextStatus = 0;
    switch (pedalID) {
      case 0: nextStatus = !pedalStatus1;
              pedalStatus1 = !pedalStatus1;
              break;
      case 1: nextStatus = !pedalStatus2;
              pedalStatus2 = !pedalStatus2;
              break;
      case 2: nextStatus = !pedalStatus3;
              pedalStatus3 = !pedalStatus3;
              break;
    }
    
    //F0 52 00 61 31 00 (pID) 00 01 (on) 00 F7
    updateLedPedals();
    uint8_t sysexData[] = { 0xF0, 0x52, 0x00, 0x61, 0x31, pedalID, 0x00, nextStatus, 0x00,  0xF7 };
    uint8_t scode = Midi.SendSysEx(sysexData, sizeof(sysexData));
    delay(150);
  }
  else { Serial.println(" - - > SERIAL NOT WORKINGG!"); }
}

void SendSysexUpdatePedals() {
  Usb.Task();
  if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
    // F0 52 00 61 29 F7
    uint8_t sysexData[] = { 0xF0, 0x52, 0x00, 0x61, 0x29, 0xF7 };
    uint8_t scode = Midi.SendSysEx(sysexData, sizeof(sysexData));
  }
  else { Serial.println(" - - > SERIAL NOT WORKINGG!"); }
}

void updateLedsNumber(byte currentPreset) {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write("pos");        // sends 3 bytes
  Wire.write(currentPreset);              // sends 1 byte
  Wire.endTransmission();    // stop transmitting
}

void MIDI_poll() {
  uint8_t recvBuf[MIDI_EVENT_PACKET_SIZE];
  uint8_t rcode = 0; //return code
  uint16_t rcvd;
  uint8_t readPtr = 0;
  
  rcode = Midi.RecvData(&rcvd, recvBuf);

  //data check
  if (rcode != 0) return;
  if ( recvBuf[0] == 0 && recvBuf[1] == 0 && recvBuf[2] == 0 && recvBuf[3] == 0 ) {
    return;
  }

  uint8_t *p = recvBuf;
  
  while (readPtr < MIDI_EVENT_PACKET_SIZE)  {
    if (*p == 0 && *(p + 1) == 0) break; //data end
    uint8_t outbuf[3];
    uint8_t rc = xtractSysExData(p, outbuf);

    if ( rc == 0 ) {
      p++;
      Midi.lookupMsgSize(*p);
      Serial.print(",  p: ");
      Serial.print(*p, HEX);       Serial.print(" ");
      Serial.print(*(p+1), HEX); Serial.print(" ");
      Serial.println(*(p+2), HEX);
      p += 3;
    }
    else {
      boolean endOfMsg = false;
      
      for (byte i = 1; i < rc; ++i) {
        if (*(p+i) == 0xF0) indexSysExMsg = 0; // Start of a SysEx message (0xF0)
        sysExMsg[indexSysExMsg] = *(p+i);
        indexSysExMsg = min(indexSysExMsg + 1, SYSEX_MSG_LENGTH);
        if (*(p+i) == 0xF7) endOfMsg = true; // End of a SysEx message (0xF7)
      }
      
      if (endOfMsg) processSysExMsg();
            
      p += 4;
    }    
    readPtr += 4;
  }
}

#define SYSEX_MSG_TYPE_POSITION 4
#define SYSEX_MSG_TYPE_UPDATE 0x28
#define SYSEX_MSG_TYPE_DISABLE_PEDAL 0x31

void processSysExMsg() {
  #ifdef DEBUG
    Serial.println("");
    for (uint8_t i = 0; i < sizeof(sysExMsg); ++i) {
      char buffer[3];
      sprintf (buffer, "%02x", sysExMsg[i]);
      Serial.print(buffer); Serial.print(" ");
      if ((i+1) % 16 == 0) Serial.println("");
    }
  #endif
  
  if (sysExMsg[SYSEX_MSG_TYPE_POSITION] == SYSEX_MSG_TYPE_UPDATE) {
    pedalStatus1 = sysExMsg[6] & 1;
    pedalStatus2 = sysExMsg[26] & 1;
    pedalStatus3 = sysExMsg[47] & 1;
    Serial.print("pedalStatus1"); Serial.println(pedalStatus1);
    Serial.print("pedalStatus2"); Serial.println(pedalStatus2);
    Serial.print("pedalStatus3"); Serial.println(pedalStatus3);
    updateLedPedals();
  }
  else if (sysExMsg[SYSEX_MSG_TYPE_POSITION] == SYSEX_MSG_TYPE_DISABLE_PEDAL &&
          sysExMsg[6] == 0 && sysExMsg[7] == 0 && sysExMsg[8] == 0) {
    Serial.print("pedalStatus DISABLE"); Serial.println(sysExMsg[5]);
    switch(sysExMsg[5]) {
      case 0: pedalStatus1 = 0;
              break;
      case 1: pedalStatus2 = 0;
              break;
      case 2: pedalStatus3 = 0;
              break;
    }
    updateLedPedals();
  }  
}


// Poll USB MIDI Controler and send to serial MIDI
byte MIDI_poll_getCurrentPreset()
{
  uint8_t size;
  uint8_t recvBuf[MIDI_EVENT_PACKET_SIZE];
  uint8_t rcode = 0;     //return code
  uint16_t rcvd;
  uint8_t readPtr = 0;

  byte currentPreset = 100;
  
  rcode = Midi.RecvData( &rcvd, recvBuf);

  //data check
  if (rcode != 0) return;
  if (recvBuf[0] == 0 && recvBuf[1] == 0 && recvBuf[2] == 0 && recvBuf[3] == 0) {
    return ;
  }

  uint8_t *p = recvBuf;
  while (readPtr < MIDI_EVENT_PACKET_SIZE)  {
    if (*p == 0 && *(p + 1) == 0) break; //data end

    uint8_t outbuf[3];
    uint8_t rc = Midi.extractSysExData(p, outbuf);
    
    if ( rc == 0 ) {
      p++;
      size = Midi.lookupMsgSize(*p);
      
      Serial.print(">  size: ");
      Serial.print(size);
      Serial.print(",  p: ");
      Serial.print(*p, HEX);       Serial.print(" ");
      Serial.print(*(p+1), HEX); Serial.print(" ");
      Serial.println(*(p+2), HEX);
      if (*p == 0xC0) currentPreset = *(p+1);
      p += 3;
    }
    readPtr += 4;
  }
  Serial.println("");
  
  return currentPreset;
}

// UPDATE LED PEDALS
void updateLedPedals() {
  digitalWrite(outPin1, pedalStatus1);
  digitalWrite(outPin2, pedalStatus2);
  digitalWrite(outPin3, pedalStatus3);
}

// Delay time (max 16383 us)
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime)
{
  uint32_t t3;

  if ( t1 > t2 ) {
    t3 = (0xFFFFFFFF - t1 + t2);
  } else {
    t3 = t2 - t1;
  }

  if ( t3 < delayTime ) {
    delayMicroseconds(delayTime - t3);
  }
}

// LOADING LEDS
void loadingLeds(int timeLength) {
  for (byte j = 0; j < timeLength / 300; ++j) {
    digitalWrite(outPinErr, HIGH);
    digitalWrite(outPin1, HIGH);
    delay(75);
    
    digitalWrite(outPinErr, LOW); 
    digitalWrite(outPin2, HIGH);
    digitalWrite(outPin1, LOW);
    delay(75);

    digitalWrite(outPinErr, LOW); 
    digitalWrite(outPin3, HIGH);
    digitalWrite(outPin2, LOW);
    delay(75);

    digitalWrite(outPinErr, LOW); 
    digitalWrite(outPin2, HIGH);
    digitalWrite(outPin3, LOW);
    delay(75);
    digitalWrite(outPin2, LOW);
  }
}

void errorIndication() {
  digitalWrite(outPinErr, HIGH);
  updateLedsNumber(ERROR_NUM);
  Serial.println("> > ERROR < <");
  while(true);
}

// EXTRACT SYSEX DATA
uint8_t xtractSysExData(uint8_t *p, uint8_t *buf)
{
  uint8_t rc = 0;
  uint8_t cin = *(p) & 0x0f;

  //SysEx message?
  if( (cin & 0xc) != 4 ) return rc;
  switch(cin) {
      case 4:
      case 7:
          *buf++ = *(p+1);
          *buf++ = *(p+2);
          *buf++ = *(p+3);
          *buf++ = *(p+4);
          rc = 4;
          break;
      case 6:
          *buf++ = *(p+1);
          *buf++ = *(p+2);
          *buf++ = *(p+3);
          rc = 3;
          break;
      case 5:
          *buf++ = *(p+1);
          *buf++ = *(p+2);
          rc = 2;
          break;
      default:
          break;
  }
  return(rc);
}
