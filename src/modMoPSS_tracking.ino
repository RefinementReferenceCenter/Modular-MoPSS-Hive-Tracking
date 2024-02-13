// - PJRC Teensy 4.1 (with ethernet)

#include <Wire.h>       //I2C communication

//----- declaring variables ----------------------------------------------------

//I2C addresses
//maximum number of RFID modules to support, when altering also change I2C address array to fit!
const uint8_t maxReaderPairs = 10;
//RFID reader pairs
const uint8_t RFIDreader[maxReaderPairs][2] = {  //for 10 reader pair addresses
  {0x08,0x09},
  {0x0a,0x0b},
  {0x0c,0x0d},
  {0x0e,0x0f},
  {0x10,0x11},
  {0x12,0x13},
  {0x14,0x15},
  {0x16,0x17},
  {0x18,0x19},
  {0x1a,0x1b}};

//Buttons
const int buttons = A13;    //~1022 not pressed, ~1 left, ~323 middle, ~711 right

//LEDs
const int errorLED = 32;
const int statusLED = 31;

//RFID
int32_t reader1freq[maxReaderPairs] = {};    //saves resonant frequency measured at bootup
int32_t reader2freq[maxReaderPairs] = {};

//##############################################################################
//#####   U S E R   C O N F I G  ###############################################
//##############################################################################

//active reader pairs == amount of RFID modules in use
const uint8_t arp = 3;

//if set to 1, the MoPSS prints what is written to uSD to Serial as well.
const uint8_t is_testing = 1;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
  //----- Serial & I2C ---------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1){
    //while(!Serial); //wait for serial connection
    Serial.println("alive");
  }
  
  //start I2C
  Wire.begin();
  Wire.setClock(100000);
  
  //----- Buttons & Fans & LEDs ------------------------------------------------
  pinMode(buttons,INPUT);
  pinMode(statusLED,OUTPUT);
  pinMode(errorLED,OUTPUT);
  
  //----- Setup RFID readers ---------------------------------------------------
  //measure resonant frequency and confirm/repeat on detune
  
  while(1){
    for(uint8_t r = 0;r < arp;r++){   //iterate through all active reader pairs
      uint8_t RFIDmodulestate = 0;
      
      while(RFIDmodulestate == 0){
        reader1freq[r] = fetchResFreq(RFIDreader[r][0]);
        Serial.println(reader1freq[r]);
        
        if((abs(reader1freq[r] - 134200) >= 1000)){
          uint8_t buttonpress = getButton();
          if(buttonpress == 1) RFIDmodulestate = 1;
        }
        
        reader2freq[r] = fetchResFreq(RFIDreader[r][1]);
        Serial.println(reader2freq[r]);
        
        if((abs(reader2freq[r] - 134200) >= 1000)){
          uint8_t buttonpress = getButton();
          if(buttonpress == 1) RFIDmodulestate = 1;
        }
      }
    }
  }

} //end of setup

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){
  
  
} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//sets mode of the RFID reader 2 = RFID mode (retun tags), 3 = measure mode (return resonant frequency)
void setReaderMode(uint8_t reader,uint8_t mode){
  uint8_t send_status = 1;
  while(send_status != 0){
    Wire.beginTransmission(reader);
    Wire.write(mode);
    send_status = Wire.endTransmission();
  }
}

//Query reader for additional information --------------------------------------
uint32_t fetchResFreq(uint8_t reader){
  setReaderMode(reader,3); //set to frequency measure mode and perform measurement
  delay(1500);             //frequency measurement takes about >=1.1 seconds

  //fetch measured frequency
  uint32_t resfreq = 0;
  uint8_t rcv[4];
  Wire.requestFrom(reader,4,1); //request frequency
  uint8_t n = 0;
  while(Wire.available()){
    rcv[n] = Wire.read();
    n++;
  }
  //assemble from array to 32bit variable
  resfreq |= (rcv[3] << 24);
  resfreq |= (rcv[2] << 16);
  resfreq |= (rcv[1] <<  8);
  resfreq |= (rcv[0] <<  0);

  //leave measure mode
  setReaderMode(reader,2);  //set to tag-transmitting mode

  return resfreq;
}

//waits and returns which button (1,2,3) was pressed ---------------------------
uint8_t getButton(){
  uint16_t input = 1023;
  while(input > 850){
    input = analogRead(buttons);
    delay(50);
  }
  if(input <= 150) return 1;
  if(input > 150 && input <= 450) return 2;
  if(input > 450 && input <= 850) return 3;
}
