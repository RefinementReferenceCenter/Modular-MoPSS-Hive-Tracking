//------------------------------------------------------------------------------
/*
--- Adafruit ItsyBitsy M0 pin mapping - Hardware Revision v2.0 ---

 A0- [JP 9] IR 2 door 2
 A1- [JP10] IR 1 door 1
 A2- [JP 2]
 A3- [JP 3]
 A4- [JP 4]
 A5- [JP 5]

 D2-
 D3-
 D4- SD chip select
 D5-
 D7-
 D9-
D10- [JP 8] IR 3 middle
D11- [JP 7] WiFi Busy
D12- [JP 6] WiFi Reset
D13- WiFi CS

--- Experimental Setup ---

^^^^^^^\                                   /^^^^^^^^
       |                                  |
h c    |     |R|                  |R|     |    t  c
o a  ––|–––––|F|––––––––––––––––––|F|–––––|––  e  a
m g    |     |I|                  |I|     |    s  g
e e  ––|–––––|D|––––––––––––––––––|D|–––––|––  t  e
       |     |1|                  |2|     |
       |                                  |
_______/      |----- 10cm ----|           \________

*/

//------------------------------------------------------------------------------
#include <Wire.h>             //I2C communication
#include <SD.h>               //Access to SD card
//#include <RTCZero.h>          //realtimeclock on MKR Boards
#include <RTClib.h>   //provides softRTC as a workaround
#include <WiFiNINA.h>         //Wifi Chipset modified version from Adafruit
#include <Adafruit_DotStar.h> //controlling the onboard dotstar RGB LED

//----- declaring variables ----------------------------------------------------
//Current Version of the program
//##############################################################################
//##############################################################################
const char HARDWARE_REV[] = "v2.0";
//##############################################################################
//##############################################################################

//Sensors
const int sensor1 = A1; //IR 1 door 1
const int sensor2 = A0; //IR 2 door 2
const int sensor3 = 10; //IR 3 middle

//LEDs
volatile uint8_t ledstate = LOW;
Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

//SD
const uint8_t SDcs = 4; //ChipSelect pin for SD (SPI)
File dataFile;          //create file object

//WiFi
char ssid[] = "Buchhaim";           //network name
char pass[] = "2416AdZk3881QPnh+";  //WPA key
int status = WL_IDLE_STATUS;        //initialize for first check if wifi up
#define SPIWIFI SPI                 //the SPI port

//RTC - Real Time Clock
const uint8_t GMT = 1;  //current timezone (Winterzeit)
//RTCZero rtc;            //create rtc object
RTC_Millis rtc;         //create rtc object based off millis() timer

//RFID
const uint8_t reader1 = 0x08;     //I2C address RFID module 1
const uint8_t reader2 = 0x09;     //I2C address RFID module 2

unsigned long RFIDtime;           //used to measure time before switching to next antenna
uint8_t RFIDtoggle = 0;           //flag used to switch to next antenna

uint8_t tag[6] = {};                  //global variable to store returned tag data (limitation of C to return arrays)
uint8_t tag1_present = 0;             //flag that indicates if tag was present during read cycle
uint8_t tag2_present = 0;
uint8_t reader1_cycle = 0;            //toggles flag if a read cycle has just happened (not automatically cleared)
uint8_t reader2_cycle = 0;
uint8_t currenttag1[6] = {};          //saves id of the tag that was read during the current read cycle
uint8_t currenttag2[6] = {};
uint8_t lasttag1[6] = {};             //saves id of the tag that was read during the previous read cycle
uint8_t lasttag2[6] = {};

//TODO: test switching modes
uint8_t RFIDmode = 1;           //select mode to operate in: 1-alternate, 2-reader1, 3-reader2
uint8_t RFIDmode_firstrun = 1;  //to make sure the correct reader is turned on/off

//Experiment variables
unsigned long starttime;      //start of programm
unsigned long rtccheck_time;  //time the rtc was checked last

//##############################################################################
//#####   U S E R   C O N F I G  ###############################################
//##############################################################################

//if set to 1, the MoPSS will wait for a wifi connection and synchronization with network time before continuing
const uint8_t enable_wifi = 1;

//if set to 1, the MoPSS will wait until a serial connection via USB is established
//before continuing. Also prints what is written to uSD to Serial as well.
const uint8_t is_testing = 0;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup()
{
  //----- DEBUGGING ------------------------------------------------------------
  pinMode(7,OUTPUT); //to allow port toggle for timing purposes
  
  //Set up RGB LED on board, and turn it off -----------------------------------
  strip.begin(); //Initialize pins for output
  strip.show();  //Turn all LEDs off ASAP
  
  //----- communication --------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1)
  {
    //while(!Serial); //wait for serial connection
  }
  
  //start I2C
  Wire.begin(); //atsamd can't multimaster
  
  //disable RFID readers and wait until they acknowledge
  disableReader(reader1);
  disableReader(reader2);
  
  //----- WiFi -----------------------------------------------------------------
  if(enable_wifi == 1)
	{
		Serial.println("----- WiFi Setup -----"); // check if the WiFi module work
    
    WiFi.setPins(13, 11, 12, -1, &SPIWIFI);
    
		if(WiFi.status() == WL_NO_MODULE)
    {
			Serial.println("WiFi chip not working/disconnected, program stopped!");
			criticalerror(); //don't continue
		}
    
    //attempt to connect to WiFi network:
    Serial.print("Connecting to SSID: ");
    Serial.println(ssid);
    uint8_t numberOfTries = 0;
    while(status != WL_CONNECTED)
    {
      numberOfTries++;
      Serial.print("Waiting to connect, attempt No.: ");
      Serial.println(numberOfTries);

      //Connect to WPA/WPA2 network
      status = WiFi.begin(ssid, pass);

      //wait 10 seconds for each connection attempt and slowly blink LED:
      delay(2000);
      strip.setPixelColor(0,0,0,0);
      strip.show();
      delay(500);
      strip.setPixelColor(0,255,255,255);
      strip.show();
      delay(2000);
      strip.setPixelColor(0,0,0,0);
      strip.show();
      delay(500);
      strip.setPixelColor(0,255,255,255);
      strip.show();
      delay(2000);
      strip.setPixelColor(0,0,0,0);
      strip.show();
      delay(500);
      strip.setPixelColor(0,255,255,255);
      strip.show();
      delay(2000);
      strip.setPixelColor(0,0,0,0);
      strip.show();
      delay(500);
      strip.setPixelColor(0,255,255,255);
      strip.show();
    }
    Serial.println("Successfully connected!");
    
    //----- Real Time Clock ------------------------------------------------------
    Serial.println("----- RTC Setup -----");
    
    strip.setPixelColor(0,255,255,255);
    strip.show();
    
    //rtc.begin()
    //set RTC to 00:00 1.1.2000
    rtc.begin(DateTime(946684800));
    unsigned long epoch = 0; //stores the time in seconds since beginning
    
    //repeatedly contact NTP server
    Serial.println("Attempting to reach NTP server");
    numberOfTries = 0;
    while(epoch == 0)
    {
      Serial.print("Attempt No. ");
      Serial.println(numberOfTries);
      numberOfTries++;
      delay(500);
      strip.setPixelColor(0,0,0,0);
      strip.show();
      delay(500);
      strip.setPixelColor(0,255,255,255);
      strip.show();

      epoch = WiFi.getTime();
    }
    
    //rtc.setEpoch(epoch); //set time received from ntp server
    rtc.adjust(DateTime(epoch));
    
    Serial.print("Success! Time received: ");
    Serial.println(nicetime());
    
    strip.setPixelColor(0,0,0,0);
    strip.show();
    
    //disable wifi module after fetching time to conserve power (~83mA)
    WiFi.end();
  } //enablewifibracket
  
  //Set time to zero if WiFi isn't enabled
  if(enable_wifi == 0)
  {
    //rtc.begin();
    rtc.begin(DateTime(946684800));
    Serial.println("WiFi disabled, setting time to 0");
    //rtc.setEpoch(0); //1.1.2000 00:00:00 946684800
    WiFi.end(); //disable wifi module
  }
  
  //----- Setup SD Card --------------------------------------------------------
  Serial.println("----- SD-Card Setup -----");
  // see if the card is present and can be initialized:
  if (!SD.begin(SDcs))
  {
    Serial.println("Card failed, or not present, program stopped!");
    //Stop program if uSD was not detected/faulty (needs to be FAT32 format):
    criticalerror();
  }
  else
  {
    Serial.println("SD card initialized successfully!");
  }
  
  //----- Setup log file, and write initial configuration ----------------------
  //open file, or create if empty
  dataFile = SD.open("RFIDLOG.TXT", FILE_WRITE);
  DateTime now = rtc.now();
  
  //get experiment start time
  starttime = now.unixtime();
  
  //TODO: add option to query RFID reader for version and resonant frequency
  //write current version to SD and some other startup/system related information
  dataFile.println("");
  dataFile.print("# Modular MoPSS Hive version: ");
  dataFile.println("not yet implemented");

  dataFile.print("# RFID Module 1 version: ");
  dataFile.println("not yet implemented");
  dataFile.print("# RFID Module 2 version: ");
  dataFile.println("not yet implemented");
  
  dataFile.print("# System start @ ");
  dataFile.print(nicetime());
  dataFile.print(" ");
//dataFile.print(rtc.getDay());
  dataFile.print(now.day());
  dataFile.print("-");
//dataFile.print(rtc.getMonth());
  dataFile.print(now.month());
  dataFile.print("-");
//dataFile.println(rtc.getYear());
  dataFile.println(now.year());
  dataFile.print("# Unixtime: ");
  dataFile.println(starttime);
  dataFile.println();
  
  dataFile.flush();
  
} //end of setup

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop()
{
  //----------------------------------------------------------------------------
  //record RFID tags -----------------------------------------------------------
  //----------------------------------------------------------------------------
  //takes 198us from start to finishing send, and further 15.2us until rfid-module turns off RFID reader
  //>=80ms are required to cold-start a tag for a successful read (at reduced range)
  //>=90ms for full range, increasing further only seems to increase range due to noise
  //rather than requirements of the tag and coil for energizing (100ms is chosen as a compromise)
  //REG_PORT_OUTSET0 = PORT_PA21;
  //REG_PORT_OUTCLR0 = PORT_PA21;
  
  // make a string for assembling the data to log:
  String RFIDdataString = "";
  
  //switch between RFID readers
  switch(RFIDmode)
  {
    //alternately switch between both readers
    case 1:
      if((millis() - RFIDtime) >= 100)
      {
        RFIDtime = millis();
        
        if(RFIDtoggle == 1)
        {
          RFIDtoggle = 0; //toggle the toggle

          //enable reader2, disable reader1
          switchReaders(reader2,reader1);
          
          //fetch data reader1 collected during on time saved in variable: tag
          tag1_present = fetchtag(reader1, 1);
          reader1_cycle = 1;
          
          //copy received tag to current tag
          for(uint8_t i = 0; i < sizeof(tag); i++)
          {
            currenttag1[i] = tag[i];
          }
          
          //compare current and last tag
          //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
          uint8_t tag1_switch = compareTags(currenttag1,lasttag1);
          
          //create datastring that is written to uSD
          RFIDdataString = createRFIDDataString(currenttag1, lasttag1, tag1_present, tag1_switch, "R1");
          
          //copy currenttag to lasttag
          for(uint8_t i = 0; i < sizeof(currenttag1); i++)
          {
            lasttag1[i] = currenttag1[i];
          }
        }
        else
        {
          RFIDtoggle = 1; //toggle the toggle
          
          //enable reader1, disable reader2
          switchReaders(reader1,reader2);
          
          //fetch data reader2 collected during on time saved in variable: tag
          tag2_present = fetchtag(reader2, 1);
          reader2_cycle = 1;
          
          //copy received tag to current tag
          for(uint8_t i = 0; i < sizeof(tag); i++)
          {
            currenttag2[i] = tag[i];
          }
          //compare current and last tag
          //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
          uint8_t tag2_switch = compareTags(currenttag2,lasttag2);
          
          //create datastring that is written to uSD
          RFIDdataString = createRFIDDataString(currenttag2, lasttag2, tag2_present, tag2_switch, "R2");
          
          //copy currenttag to lasttag
          for(uint8_t i = 0; i < sizeof(currenttag2); i++)
          {
            lasttag2[i] = currenttag2[i];
          }
        }
      }
    break;
    
    //enable only reader1
    case 2:
      //do once
      if(RFIDmode_firstrun == 1)
      {
        RFIDmode_firstrun = 0;
        
        //dis/enable correct readers
        enableReader(reader1);
        disableReader(reader2);
      }
      
      //poll for tag reads every 40ms
      if((millis() - RFIDtime) >= 100)
      {
        RFIDtime = millis();
      
        //fetch tag
        tag1_present = fetchtag(reader1, 0);
      
        //copy received tag to current tag
        for(uint8_t i = 0; i < sizeof(tag); i++)
        {
          currenttag1[i] = tag[i];
        }
      
        //compare current and last tag
        //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag1_switch = compareTags(currenttag1, lasttag1);
      
        //create datastring that is written to uSD
        RFIDdataString = createRFIDDataString(currenttag1, lasttag1, tag1_present, tag1_switch, "R1s");
      
        //copy currenttag to lasttag
        for(uint8_t i = 0; i < sizeof(currenttag1); i++)
        {
          lasttag1[i] = currenttag1[i];
        }
      }
    break;
    
    //enable only reader2
    case 3:
      //do once
      if(RFIDmode_firstrun == 1)
      {
        RFIDmode_firstrun = 0;
        
        //dis/enable correct readers
        enableReader(reader2);
        disableReader(reader1);
      }
      
      //poll for tag reads every 40ms
      if((millis() - RFIDtime) >= 100)
      {
        RFIDtime = millis();
      
        //fetch tag
        tag2_present = fetchtag(reader2, 0);
      
        //copy received tag to current tag
        for(uint8_t i = 0; i < sizeof(tag); i++)
        {
          currenttag2[i] = tag[i];
        }
      
        //compare current and last tag
        //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag2_switch = compareTags(currenttag2,lasttag2);
      
        //create datastring that is written to uSD
        RFIDdataString = createRFIDDataString(currenttag2, lasttag2, tag2_present, tag2_switch, "R2s");
      
        //copy currenttag to lasttag
        for(uint8_t i = 0; i < sizeof(currenttag2); i++)
        {
          lasttag2[i] = currenttag2[i];
        }
      }
    break;
  }
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //do stuff (continuously) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //do other stuff (every now and then) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//write Data to log files ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  //log RFID events
  if(RFIDdataString.length() != 0) //13.7ms
	{
		//append Datastring to file
		dataFile.println(RFIDdataString);
		dataFile.flush();
		if(is_testing == 1)
		{
			Serial.println(RFIDdataString);
		}
	}

} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//Return time as string in HH:MM:SS format
String nicetime()
{
  DateTime now = rtc.now();
  
	String ntime = "";
  //int h = rtc.getHours() + GMT;
	//int m = rtc.getMinutes();
	//int s = rtc.getSeconds();
  int h = now.hour() + GMT;
  int m = now.minute();
  int s = now.second();

	if (h < 10)
	{
		ntime += "0";
		ntime += h;
	}
	else
	{
		ntime += h;
	}
	ntime += ":";

	if (m < 10)
	{
		ntime += "0";
		ntime += m;
	}
	else
	{
		ntime += m;
	}
	ntime += ":";

	if (s < 10)
	{
		ntime += "0";
		ntime += s;
	}
	else
	{
		ntime += s;
	}
	return ntime;
}

//RFID tag realted functions ---------------------------------------------------
//get ID in string format
String getID(uint8_t in[6])
{
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
  while(in64)
  {
    char c = in64 % 10;
    in64 /= 10;
    c += '0'; //add to character zero
    result = c + result; //concatenate
  }
  return result;
}

//convert byte array to char
uint16_t getCountryCode(uint8_t in[6])
{
  uint16_t countrycode = 0;
  countrycode = ((countrycode | in[5]) << 2) | ((in[4] >> 6) & 0b11);
  return countrycode;
}

//enable one reader, wait for cofirmation from reader
void enableReader(byte reader)
{
  uint8_t send_status = 1;
  while(send_status != 0)
  {
    //enable reader
    Wire.beginTransmission(reader);
    Wire.write(1);
    send_status = Wire.endTransmission();
  }
}

//disable one reader, wait for cofirmation from reader
void disableReader(byte reader)
{
  uint8_t send_status = 1;
  while(send_status != 0)
  {
    //disable reader
    Wire.beginTransmission(reader);
    Wire.write(0);
    send_status = Wire.endTransmission();
  }
}

//switch between two readers, optimized timing for minimum downtime
void switchReaders(byte readerON, byte readerOFF)
{
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

//fetch tag data from reader
byte fetchtag(byte reader, byte busrelease)
{
  //request tag-data from reader
  Wire.requestFrom(reader,6,busrelease); //address, quantity ~574uS, bus release
  int n = 0;
  while(Wire.available())
  {
    tag[n] = Wire.read();
    n++;
  }
  
  //sum received values
  int tag_sum = 0;
  for(uint8_t i = 0; i < sizeof(tag); i++)
  {
    tag_sum = tag_sum + tag[i];
  }
    
  //if tag is empty, no tag was detected
  if(tag_sum > 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//compare current and last tag
byte compareTags(byte currenttag[], byte lasttag[])
{
  //compare lasttag and currenttag if the tag changes, skipped if no tag was present before
  int tagchange = 0; //0 = no change, 1 = new tag entered, 2 = switch (2 present), 3 = tag left
  int lasttag_sum = 0;
  int currenttag_sum = 0;
  for(uint8_t i = 0; i < sizeof(lasttag); i++)
  {
    if(currenttag[i] != lasttag[i]) //if diff between current and last tag, something changed
    {
      //check if arrays are empty by summing all values
      for(uint8_t j = 0; j < sizeof(lasttag); j++)
      {
        lasttag_sum = lasttag_sum + lasttag[j];
        currenttag_sum = currenttag_sum + currenttag[j];
      }
      
      //if lasttag is empty but not currenttag: 1 = new tag entered
      if(lasttag_sum == 0)
      {
        tagchange = 1;
      }
      //if lasttag wasn't empty nad currenttaf isn't either, tags switched (two present, one left)
      if((lasttag_sum != 0) && (currenttag_sum != 0))
      {
        tagchange = 2;
      }
      //if currenttag is empty, but not last tag, 3 = tag left
      if(currenttag_sum == 0)
      {
        tagchange = 3;
      }
      break;
    }
  }

  //return how (if) the tag changed
  return(tagchange);
}

//create string that is later saved to uSD
String createRFIDDataString(byte currenttag[], byte lasttag[], byte currenttag_present, int tagchange, String identifier)
{
  String dataString;
  DateTime now = rtc.now();
  
  //get country code and tag ID
  int ctCC = getCountryCode(currenttag);
  int ltCC = getCountryCode(lasttag);

  String ctID = getID(currenttag);
  String ltID = getID(lasttag);

  //save tag data to dataString which is written to SD
  //tag left (3) or switch (2)
  if((tagchange == 2) || (tagchange == 3))
  {
    dataString += identifier;
    dataString += ",";
    //dataString += rtc.getEpoch();
    dataString += now.unixtime();
    dataString += ",";
    dataString += ltCC;
    dataString += "_";
    dataString += ltID;
    dataString += ",";
    dataString += millis();
    dataString += ",X";
  }
  
  //insert newline when a switch happens
  if(tagchange == 2)
  {
    dataString += "\n";
  }
  
  //new tag entered (1) or switch (2)
  if((tagchange == 1) || (tagchange == 2))
  {
    dataString += identifier;
    dataString += ",";
    //dataString += rtc.getEpoch();
    dataString += now.unixtime();
    dataString += ",";
    dataString += ctCC;
    dataString += "_";
    dataString += ctID;
    dataString += ",";
    dataString += millis();
    dataString += ",E";
  }
  return dataString;
}

//critical error, flash LED SOS ------------------------------------------------
void criticalerror()
{
  while(1)
  {
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(300);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(300);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(300);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(500);
    
  }
}
