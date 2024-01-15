/*------------------------------------------------------------------------------
- PJRC Teensy 4.1 (with ethernet) pin mapping - Hardware Revision v7.1

- dual-infrared 4-pin lightbarrier connectors (S|S|GND|+12V)
D36,D37 - X1
A14,A15 - X2
A11,A10 - X3
A16,A17 - X4
A1,A0   - X5
A6,A7   - X6
A8,A9   - X7
D3,D2   - X8

- multi-purpose 3-pin connectors (Signal|GND|+12V)
D9  - J4
D8  - J5
D7  - J6
D6  - J7

- fan-connectors 2-pin, 12V transistor-switched (GND|+12V)
D29 - F1
D28 - F2

D32 - ERR LED   Error LED, used for various error states
D31 - STAT LED  Status LED, can be used to signal stuff
A13 - B1,B2,B3  Input from the three buttons on the board

D11 - MOSI
D12 - MISO
D13 - SCK
D18 - SDA
D19 - SCL

--- Experimental Setup ---

 ^^^^^^^\      1 Reader Pair     /^^^^^^^
        |                        |
   c    |     |R|        |R|     |    c
   a  ––|–––––|F|––––––––|F|–––––|––  a
   g    |     |I|        |I|     |    g
   e  ––|–––––|D|––––––––|D|–––––|––  e
   1    |     |1|        |2|     |    2
        |                        |
 _______/                        \_______

*///----------------------------------------------------------------------------
#include <TimeLib.h>  //Manage Real Time CLock
#include <Wire.h>     //I2C communication
#include <SdFat.h>    //Access SD Cards
#include <U8g2lib.h>  //for SSD1306 OLED Display

//----- declaring variables ----------------------------------------------------
//Current Version of the program
const char SOFTWARE_REV[] = "v1.0.0";

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

const uint8_t oledDisplay = 0x78; //I2C address oled display

//Buttons
const int buttons = A13;    //~1022 not pressed, ~1 left, ~323 middle, ~711 right

//LEDs
const int errorLED = 32;
const int statusLED = 31;

//SD cardsModules
#define SD_FAT_TYPE 3
#define SPI_CLOCK SD_SCK_MHZ(16)
const uint8_t SDcs = 10;    //Chip Select External SD
SdFs SD;
FsFile dataFile;
const uint8_t SDBcs = 44;   //Chip Select Internal SD
SdFs SDb;
FsFile dataFileBackup;

//Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE,23,22); //def,reset,SCL,SDA
uint32_t displaytime = 0;    //stores millis to time display refresh
uint8_t displayon = 1;       //flag to en/disable display

//RFID
uint32_t RFIDtime[maxReaderPairs];       //used to measure time before switching to next antenna
uint8_t RFIDtoggle[maxReaderPairs];      //flag used to switch to next antenna

uint8_t tag[7] = {};                         //global variable to store returned tag data. 0-5 tag, 6 temperature
uint8_t currenttag1[maxReaderPairs][7] = {}; //saves id of the tag that was read during the current read cycle
uint8_t currenttag2[maxReaderPairs][7] = {};
uint8_t lasttag1[maxReaderPairs][7] = {};    //saves id of the tag that was read during the previous read cycle
uint8_t lasttag2[maxReaderPairs][7] = {};

int32_t reader1freq[maxReaderPairs] = {};    //saves resonant frequency measured at bootup
int32_t reader2freq[maxReaderPairs] = {};

//Experiment variables
uint32_t starttime;        //start of programm
uint32_t rtccheck_time;    //time the rtc was checked last

//##############################################################################
//#####   U S E R   C O N F I G  ###############################################
//##############################################################################

//active reader pairs == amount of RFID modules in use
const uint8_t arp = 3;

//Give each reader pair an identifier character that is _unique_ across the _whole_ experiment!
//output in log will show RFID reads like this: R?1 and R?2 where ? is the chosen identifier
//It is only necessary to asign identifiers equal to the amount of active reader pairs
const char RFIDreaderNames[maxReaderPairs + 1] = {'A','B','C','?','?','?','?','?','?','?'}; //Single character only!

//if set to 1, the MoPSS prints what is written to uSD to Serial as well.
const uint8_t is_testing = 1;

//write additional information to SD card
//debug = 0 includes door movements, RFID tags
//debug = 1 include regular status of the IR barriers
const uint8_t debug = 0;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
  //----- USER CONFIG (CONTINUED) ----------------------------------------------

  //----- communication --------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1){
    //while(!Serial); //wait for serial connection
    Serial.println("alive");
  }
  
  //start I2C
  Wire.begin();

  //----- Buttons & Fans -------------------------------------------------------
  pinMode(buttons,INPUT);

  //----- Sensors --------------------------------------------------------------

  //----- Real Time Clock ------------------------------------------------------
  setSyncProvider(getTeensy3Time);

  //----- Display --------------------------------------------------------------
  oled.setI2CAddress(oledDisplay);
  oled.begin();
  oled.setFont(u8g2_font_6x10_mf); //set font w5 h10

  //----- Setup RFID readers ---------------------------------------------------
  //measure resonant frequency and confirm/repeat on detune
  for(uint8_t r = 0;r < arp;r++){   //iterate through all active reader pairs
    OLEDprint(0,0,1,1,">>> RFID Setup <<<");
    
    char reader1[4] = {'R',RFIDreaderNames[r],'1'};
    OLEDprint(1,0,0,1,reader1);
    OLEDprint(1,3,0,1,":");
    
    char reader2[4] = {'R',RFIDreaderNames[r],'2'};
    OLEDprint(2,0,0,1,reader2);
    OLEDprint(2,3,0,1,":");
    
    uint8_t RFIDmodulestate = 0;
    
    while(RFIDmodulestate == 0){
      reader1freq[r] = fetchResFreq(RFIDreader[r][0]);
      OLEDprintFraction(1,5,0,1,(float)reader1freq[r]/1000,3);
      OLEDprint(1,12,0,1,"kHz");
      reader2freq[r] = fetchResFreq(RFIDreader[r][1]);
      OLEDprintFraction(2,5,0,1,(float)reader2freq[r]/1000,3);
      OLEDprint(2,12,0,1,"kHz");

      if((abs(reader1freq[r] - 134200) >= 1000) || (abs(reader2freq[r] - 134200) >= 1000)){
        OLEDprint(4,0,0,1,"Antenna detuned!");
        OLEDprint(5,0,0,1,"CONFIRM");
        OLEDprint(5,14,0,1,"REPEAT");
        uint8_t buttonpress = getButton();
        if(buttonpress == 1) RFIDmodulestate = 1;
      }
      else{
        OLEDprint(5,0,0,1,"-Done");
        delay(1000);  //to give time to actually read the display
        RFIDmodulestate = 1;
      }
    }
  }
  
  //----- Setup SD Card --------------------------------------------------------
  //Stop program if uSDs are not detected/faulty (needs to be FAT/FAT32/exFAT format)
  OLEDprint(0,0,1,1,">>>  uSD  Setup  <<<");
  OLEDprint(1,0,0,1,"SD EXternal:"); //see if the cards are present and can be initialized
  OLEDprint(2,0,0,1,"SD INternal:");
  
  //SD card external (main, for data collection)
  if(!SD.begin(SDcs)){
    OLEDprint(1,13,0,1,"FAIL!");
    OLEDprint(5,0,0,1,"PROGRAM STOPPED");
    criticalerror();
  }
  else{
    Serial.println("External SD card initialized successfully!");
    OLEDprint(1,13,0,1,"OK!");
  }
  //SD card internal (Backup)
  if(!SDb.begin(SdioConfig(FIFO_SDIO))){ //internal SD Card
    OLEDprint(2,13,0,1,"FAIL!");
    OLEDprint(5,0,0,1,"PROGRAM STOPPED");
    criticalerror();
  }
  else{
    Serial.println("Internal SD card initialized successfully!");
    OLEDprint(2,13,0,1,"OK!");
  }
  delay(1000);

  //----- Setup log file, and write initial configuration ----------------------
  dataFile = SD.open("RFIDLOG.TXT", FILE_WRITE); //open file, or create if empty
  dataFileBackup = SDb.open("RFIDLOG_BACKUP.TXT", FILE_WRITE);
  
  starttime = now(); //get experiment start time

  //write current version to SD and some other startup/system related information
  dataFile.println("");
  dataFile.print("# Modular MoPSS Hive version: ");
  dataFile.println(SOFTWARE_REV);
  
  for(uint8_t r = 0;r < arp;r++){
    char reader1[4] = {'R',RFIDreaderNames[r],'1'};
    char reader2[4] = {'R',RFIDreaderNames[r],'2'};
    
    dataFile.print("# RFID Antenna ");
    dataFile.print(reader1);
    dataFile.print(" resonant frequency: ");
    dataFile.print(reader1freq[r]);
    dataFile.println(" Hz");
    dataFile.print("# RFID Antenna ");
    dataFile.print(reader2);
    dataFile.print(" resonant frequency: ");
    dataFile.print(reader2freq[r]);
    dataFile.println(" Hz");
  }
  
  dataFile.print("# debug level: ");
  dataFile.println(debug);
  dataFile.print("# System start @ ");
  dataFile.print(nicetime(starttime));
  dataFile.print(" ");
  dataFile.print(day(starttime));
  dataFile.print("-");
  dataFile.print(month(starttime));
  dataFile.print("-");
  dataFile.println(year(starttime));
  dataFile.print("# Unixtime: ");
  dataFile.println(starttime);
  dataFile.println();

  dataFile.flush();

  //and same for backup SD
  dataFileBackup.println("");
  dataFileBackup.print("# Modular MoPSS Hive version: ");
  dataFileBackup.println(SOFTWARE_REV);
  
  for(uint8_t r = 0;r < arp;r++){
    char reader1[4] = {'R',RFIDreaderNames[r],'1'};
    char reader2[4] = {'R',RFIDreaderNames[r],'2'};
    
    dataFileBackup.print("# RFID Antenna ");
    dataFileBackup.print(reader1);
    dataFileBackup.print(" resonant frequency: ");
    dataFileBackup.print(reader1freq[r]);
    dataFileBackup.println(" Hz");
    dataFileBackup.print("# RFID Antenna ");
    dataFileBackup.print(reader2);
    dataFileBackup.print(" resonant frequency: ");
    dataFileBackup.print(reader2freq[r]);
    dataFileBackup.println(" Hz");
  }
  
  dataFileBackup.print("# debug level: ");
  dataFileBackup.println(debug);
  dataFileBackup.print("# System start @ ");
  dataFileBackup.print(nicetime(starttime));
  dataFileBackup.print(" ");
  dataFileBackup.print(day(starttime));
  dataFileBackup.print("-");
  dataFileBackup.print(month(starttime));
  dataFileBackup.print("-");
  dataFileBackup.println(year(starttime));
  dataFileBackup.print("# Unixtime: ");
  dataFileBackup.println(starttime);
  dataFileBackup.println();

  dataFileBackup.flush();
  
  //output start log to console
  if(is_testing){
    Serial.print("# Modular MoPSS Hive version: ");
    Serial.println(SOFTWARE_REV);
    
    for(uint8_t r = 0;r < arp;r++){
      char reader1[4] = {'R',RFIDreaderNames[r],'1'};
      char reader2[4] = {'R',RFIDreaderNames[r],'2'};
      
      Serial.print("# RFID Antenna ");
      Serial.print(reader1);
      Serial.print(" resonant frequency: ");
      Serial.print(reader1freq[r]);
      Serial.println(" Hz");
      Serial.print("# RFID Antenna ");
      Serial.print(reader2);
      Serial.print(" resonant frequency: ");
      Serial.print(reader2freq[r]);
      Serial.println(" Hz");
    }
    
    Serial.print("# debug level: ");
    Serial.println(debug);
    Serial.print("# System start @ ");
    Serial.print(nicetime(starttime));
    Serial.print(" ");
    Serial.print(day(starttime));
    Serial.print("-");
    Serial.print(month(starttime));
    Serial.print("-");
    Serial.println(year(starttime));
    Serial.print("# Unixtime: ");
    Serial.println(starttime);
    Serial.println();
  }
  
  //---- setup experiment ------------------------------------------------------

} //end of setup

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){

  //create/clear strings that get written to uSD card
  String RFIDdataString = ""; //holds tag and date
  String SENSORDataString = ""; //holds various sensor and diagnostics data
  
  //----------------------------------------------------------------------------
  //record RFID tags -----------------------------------------------------------
  //----------------------------------------------------------------------------
  //>=80ms are required to cold-start a tag for a successful read (at reduced range)
  //>=90ms for full range, increasing further only seems to increase range due to noise
  //rather than requirements of the tag and coil for energizing (100ms is chosen as a compromise)
  
  for(uint8_t r = 0;r < arp;r++){
    if((millis() - RFIDtime[r]) >= 100){
      RFIDtime[r] = millis();
      
      if(RFIDtoggle[r] == 1){
        RFIDtoggle[r] = 0; //toggle the toggle
        switchReaders(RFIDreader[r][1],RFIDreader[r][0]); //enable reader2, disable reader1
        
        uint8_t tag_status = fetchtag(RFIDreader[r][0],1); //fetch data reader1 collected during on-time saved in variable: tag
        for(uint8_t i = 0; i < sizeof(tag); i++) currenttag1[r][i] = tag[i]; //copy received tag to current tag
        
        //compare current and last tag 0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag_switch = compareTags(currenttag1[r],lasttag1[r]);
        char reader[4] = {'R',RFIDreaderNames[r],'1'};
        RFIDdataString = createRFIDDataString(currenttag1[r], lasttag1[r], tag_switch, reader, RFIDdataString); //create datastring that is written to uSD
        for(uint8_t i = 0; i < sizeof(currenttag1[r]); i++) lasttag1[r][i] = currenttag1[r][i]; //copy currenttag to lasttag
      }
      else{
        RFIDtoggle[r] = 1; //toggle the toggle
        switchReaders(RFIDreader[r][0],RFIDreader[r][1]); //enable reader1, disable reader2
        
        uint8_t tag_status = fetchtag(RFIDreader[r][1],1); //fetch data reader2 collected during on-time saved in variable: tag
        for(uint8_t i = 0; i < sizeof(tag); i++) currenttag2[r][i] = tag[i]; //copy received tag to current tag
        
        //compare current and last tag 0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag_switch = compareTags(currenttag2[r],lasttag2[r]);
        char reader[4] = {'R',RFIDreaderNames[r],'2'};
        RFIDdataString = createRFIDDataString(currenttag2[r], lasttag2[r], tag_switch, reader, RFIDdataString); //create datastring that is written to uSD
        for(uint8_t i = 0; i < sizeof(currenttag2[r]); i++) lasttag2[r][i] = currenttag2[r][i]; //copy currenttag to lasttag
      }
    }
  }
  
  //----------------------------------------------------------------------------
  //update display -------------------------------------------------------------
  //----------------------------------------------------------------------------
  if((millis() - displaytime) > 1000){ //once every second
    displaytime = millis();

    //switch display on/off if button pressed
    if(analogRead(buttons) < 850){
      displayon = !displayon;
      if(!displayon){
        oled.clearBuffer();   //clear display
        oled.sendBuffer();
      }
    }

    if(displayon){
      oled.clearBuffer(); //clear display
      time_t rtctime = now(); //create nice date string
      uint8_t D = day(rtctime);
      uint8_t M = month(rtctime);
      String nDate = "";

      if(D < 10) nDate += "0";
      nDate += D;
      nDate += "-";
      if(M < 10) nDate += "0";
      nDate += M;
      nDate += "-";
      nDate += year(rtctime);

      //display current time from RTC and date
      OLEDprint(0,0,0,0,nicetime(rtctime));
      OLEDprint(0,11,0,0,nDate);

      //update display
      oled.sendBuffer();
    }
  }

  //----------------------------------------------------------------------------
	//write Data to log files ----------------------------------------------------
	//----------------------------------------------------------------------------
	//log sensor/motor events
  if(SENSORDataString.length() != 0){
    dataFile.println(SENSORDataString); //append Datastring to file
    dataFile.flush();
    dataFileBackup.println(SENSORDataString);
    dataFileBackup.flush();
    if(is_testing == 1) Serial.println(SENSORDataString); //print to serial if in testmode
  }
  //log RFID events
  if(RFIDdataString.length() != 0){
		dataFile.println(RFIDdataString);	//append Datastring to file
		dataFile.flush();
		dataFileBackup.println(RFIDdataString);
		dataFileBackup.flush();
		if(is_testing == 1) Serial.println(RFIDdataString);
	}

} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//Get Time from internal RTC (updated on program upload) -----------------------
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}

//Return time as string in HH:MM:SS format -------------------------------------
String nicetime(time_t nowtime){
	String ntime = "";
  uint8_t h = hour(nowtime);// + GMT;
  uint8_t m = minute(nowtime);
  uint8_t s = second(nowtime);

	if (h < 10) ntime += "0";
	ntime += h;
	ntime += ":";
	if (m < 10) ntime += "0";
	ntime += m;
	ntime += ":";
	if (s < 10) ntime += "0";
	ntime += s;
	return ntime;
}

//Sensors related functions ----------------------------------------------------
String createSENSORDataString(String identifier, String event, String dataString){
  time_t nowtime = now();

  if(dataString != 0) dataString += "\n"; //if datastring is not empty, add newline
  dataString += identifier;
  dataString += ",";
  dataString += nowtime;
  dataString += ",";
  dataString += "";
  dataString += ",";
  dataString += millis();
  dataString += ",";
  dataString += event;

  return dataString;
}

//Helper for printing to OLED Display (text) -----------------------------------
void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, String text){
  if(clear) oled.clearBuffer(); //clear screen 
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(text);
  if(update) oled.sendBuffer();
}

//Helper for printing to OLED Display (number) ---------------------------------
void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, int32_t number){
  if(clear) oled.clearBuffer(); //clear screen  
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(number);
  if(update) oled.sendBuffer();
}

//Helper for printing to OLED Display (number with n decimals) -----------------
void OLEDprintFraction(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, float number, uint8_t decimals){
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(number,decimals);
  if(update) oled.sendBuffer();
}

//get RFID ID in string format -------------------------------------------------
String getID(uint8_t in[7]){
  uint64_t in64 = 0;
  in64 |= (in[4] & 0b111111);
  in64 <<= 8;
  in64 |= in[3];
  in64 <<= 8;
  in64 |= in[2];
  in64 <<= 8;
  in64 |= in[1];
  in64 <<= 8;
  in64 |= in[0];

  String result = "";
  while(in64){
    char c = in64 % 10;
    in64 /= 10;
    c += '0'; //add to character zero
    result = c + result; //concatenate
  }
  return result;
}

//convert byte array to char (RFID countrycode) --------------------------------
uint16_t getCountryCode(uint8_t in[7]){
  uint16_t countrycode = 0;
  countrycode = ((countrycode | in[5]) << 2) | ((in[4] >> 6) & 0b11);
  return countrycode;
}

//get temperature in raw format ------------------------------------------------
uint8_t getTemperature(uint8_t in[7]){
  if(in[6] <= 5) return 0;
  else return in[6];
}

//get temperature in °C format -------------------------------------------------
float getTemperatureC(uint8_t in[7]){
  if(in[6] <= 5) return 0;
  else return (in[6] * 0.108296277 + 23.22566506); //can be used to return human readable temp though translation factors are based on n=1
}

//enable one reader, wait for confirmation from reader -------------------------
void enableReader(uint8_t reader){
  uint8_t send_status = 1;
  while(send_status != 0){
    Wire.beginTransmission(reader);
    Wire.write(1); //enable reader
    send_status = Wire.endTransmission();
  }
}

//disable one reader, wait for confirmation from reader ------------------------
void disableReader(uint8_t reader){
  uint8_t send_status = 1;
  while(send_status != 0){
    Wire.beginTransmission(reader);
    Wire.write(0); //disable reader
    send_status = Wire.endTransmission();
  }
}

//switch between two readers, optimized timing for minimum downtime ------------
void switchReaders(byte readerON, byte readerOFF){
  //turn on one reader
  Wire.beginTransmission(readerON);
  Wire.write(1);
  Wire.endTransmission();
  delayMicroseconds(1200); //reduces down-time of antennas since startup isn't instant
  //turn off the other
  Wire.beginTransmission(readerOFF);
  Wire.write(0);
  Wire.endTransmission();
}

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

//fetch tag data from reader ---------------------------------------------------
uint8_t fetchtag(byte reader, byte busrelease){
  Wire.requestFrom(reader,7,busrelease); //address, quantity ~574uS, bus release
  uint8_t n = 0;
  while(Wire.available()){
    tag[n] = Wire.read();
    n++;
  }

  //sum received values
  int16_t tag_sum = 0;
  for(uint8_t i = 0; i < sizeof(tag); i++){
    tag_sum = tag_sum + tag[i];
  }
  
  //return status
  if(n == 0){                     //if we didn't receive any data from reader, zero tag
    for(uint8_t i = 0;i < 7;i++) tag[i] = 0;
    return 2;
  }
  else if(tag_sum > 0) return 1;  //if we received data
  else return 0;                  //we received data, but only zeros
}

//compare current and last tag, no change 0, new tag entered 1, switch 2 (2 present), tag left 3
uint8_t compareTags(byte currenttag[], byte lasttag[]){
  uint8_t tagchange = 0; //0 = no change, 1 = new tag entered, 2 = switch (2 present), 3 = tag left
  int16_t lasttag_sum = 0;
  int16_t currenttag_sum = 0;

  for(uint8_t i = 0; i < sizeof(lasttag); i++){
    if(currenttag[i] != lasttag[i]){ //if diff between current and last tag, something changed
      for(uint8_t j = 0; j < sizeof(lasttag); j++){  //check if arrays are empty by summing all values
        lasttag_sum = lasttag_sum + lasttag[j];
        currenttag_sum = currenttag_sum + currenttag[j];
      }
      if(lasttag_sum == 0) tagchange = 1;                            //if lasttag is empty but not currenttag: 1 = new tag entered
      if((lasttag_sum != 0) && (currenttag_sum != 0)) tagchange = 2; //if lasttag wasn't empty and currenttag isn't either, tags switched (two present, one left)
      if(currenttag_sum == 0) tagchange = 3;                         //if currenttag is empty, but not last tag, 3 = tag left
      break;
    }
  }
  return(tagchange); //return how (if) the tag changed
}

//create string that is later saved to uSD -------------------------------------
String createRFIDDataString(byte currenttag[], byte lasttag[], int tagchange, char identifier[], String dataString){
  time_t nowtime = now();
  
  //get country code and tag ID for currenttag (ct) and lasttag (lt)
  int16_t ctCC = getCountryCode(currenttag);
  int16_t ltCC = getCountryCode(lasttag);
  String ctID = getID(currenttag);
  String ltID = getID(lasttag);
  //float ctT = getTemperatureC(currenttag); //Celsius is only recommended if temperature calibration is assured, otherwise convert later
  //float ltT = getTemperatureC(lasttag);
  uint8_t ctT = getTemperature(currenttag);
  uint8_t ltT = getTemperature(lasttag);
  
  //save tag data to dataString which is written to SD
  if(dataString && tagchange) dataString += "\n"; //if datastring is not empty, and tag changed, add newline
  
  if((tagchange == 2) || (tagchange == 3)){ //tag left (3) or switch (2)
    dataString += String(identifier);
    dataString += ",";
    dataString += nowtime;
    dataString += ",";
    dataString += ltCC;
    dataString += "_";
    dataString += ltID;
    dataString += ",";
    dataString += ltT;
    dataString += ",";
    dataString += millis();
    dataString += ",X";
  }
  //insert newline when a switch happens
  if(tagchange == 2){
    dataString += "\n";
  }
  //new tag entered (1) or switch (2)
  if((tagchange == 1) || (tagchange == 2)){
    dataString += String(identifier);
    dataString += ",";
    dataString += nowtime;
    dataString += ",";
    dataString += ctCC;
    dataString += "_";
    dataString += ctID;
    dataString += ",";
    dataString += ctT;
    dataString += ",";
    dataString += millis();
    dataString += ",E";
  }
  return dataString;
}

//critical error, flash LED SOS, stop everything -------------------------------
void criticalerror(){
  while(1){
    digitalWrite(errorLED,HIGH);
    delay(200);
    digitalWrite(errorLED,LOW);
    delay(200);
  }
}

//confirm with any button ------------------------------------------------------
void confirm(){
  while(analogRead(buttons) > 850){
    delay(50);
  }
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