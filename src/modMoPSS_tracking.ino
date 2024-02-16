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
#include <TimeLib.h>         //Manage Real Time CLock
#include <i2c_driver_wire.h> //I2C communication
#include <SdFat.h>           //Access SD Cards
#include <U8g2lib.h>         //for SSD1306 OLED Display
#include <QNEthernet.h>      //for ethernet

//----- declaring variables ----------------------------------------------------
//Current Version of the program
const char SOFTWARE_REV[] = "v1.0.0";

//Ethernet & NTP
using namespace qindesign::network;

constexpr uint32_t DHCPTimeout = 1'000;       //15 seconds timeout to get DHCP IP address
constexpr uint16_t NTPPort = 123;              //port for ntp requests
constexpr uint32_t EpochDiff = 2'208'988'800;  //01-Jan-1900 00:00:00 -> 01-Jan-1970 00:00:00
constexpr uint32_t EBreakTime = 2'085'978'496; //Epoch -> 07-Feb-2036 06:28:16

EthernetUDP udp; //UDP port

//NTP server (fritz.box or other) on local network is very fast and recommended
//de.pool.ntp.org took in tests about ~200ms to respond to the ntp request vs local fritz.box ~3ms
const char NTPserver[] = "fritz.box";
uint8_t ntpbuf[48];                         //ntp packet buffer
const uint32_t NTP_server_timeout = 25'000; //how long to wait for ntp server repsonse in us
double server_res_ms; //time it takes for the NTP server to respond to the request

double RTC_drift_ms;                 //difference between RTC and NTP time
const uint8_t drift_array_size = 31; //must be odd (or adjust median calculation)
double RTC_drift_array[drift_array_size]; //collects the last n RTC drift values (in sec. per sec.) to get a median drift value
uint8_t median_ok = 0;               //flag that the array was filled at least once so median/mean is usable
double RTC_drift_ms_median;          //median value of drift array
double RTC_drift_median_mean;        //average of median array
uint8_t RTC_drift_it = 0;            //counter to iterate through drift array

elapsedMillis NTPsynctime;   //time since last ntp sync in ms
double last_sync;            //last time NTP synced successfully
double second_last_sync;     //second last successfull sync time
uint8_t NTP_sync_failed = 0; //counts how often the NTP sync failed
uint8_t force_sync = 0;      //force NTP sync flag
double burst_array[3];       //stores RTC drift values of multiple syncs to get more accurate drift values
double NTP_timestamps[5];    //stores the timestamps of the NTP packet, local send/rec, server send/rec and adjusted time
const double clock_set_offset = 0.000939; //time it takes the RTC to be set (RTC is briefly stopped while being set)


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
uint8_t displayon = 1;       //flag to en/disable display
elapsedMillis displaytime;   //time since last display update
uint8_t page = 0;            //currently displayed page
int16_t maxpages;            //total number of pages

//RFID
uint32_t RFIDtime[maxReaderPairs];  //used to measure time before switching to next antenna
uint8_t RFIDtoggle[maxReaderPairs]; //flag used to switch to next antenna
uint8_t globalRFIDtoggle = 0;
elapsedMillis globalRFIDtime;       //global timer to switch antennas in pairs

uint8_t tag[7] = {};                         //global variable to store returned tag data. 0-5 tag, 6 temperature
uint8_t currenttag1[maxReaderPairs][7] = {}; //saves id of the tag that was read during the current read cycle
uint8_t currenttag2[maxReaderPairs][7] = {};
uint8_t lasttag1[maxReaderPairs][7] = {};    //saves id of the tag that was read during the previous read cycle
uint8_t lasttag2[maxReaderPairs][7] = {};

uint8_t latestreadtag1[maxReaderPairs][7] = {}; //stores the last recorded tag, currenttag/lasttag are cleared when no tag was read
uint8_t latestreadtag2[maxReaderPairs][7] = {};
uint32_t latest_tagtime1[maxReaderPairs] = {};  //stores the time of the last recorded tag, currenttag/lasttag are cleared when no tag was read
uint32_t latest_tagtime2[maxReaderPairs] = {};

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

//Interval at which the RTC should be updated, either via NTP or offline if enough data is available
const uint16_t syncinterval = 30; //in seconds

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
  
  //----- Serial & I2C ---------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1){
    //while(!Serial); //wait for serial connection
    Serial.println("alive");
  }
  
  //start I2C
  Wire.begin();
  //Wire.setClock(100000);
  
  //----- Buttons & Fans & LEDs ------------------------------------------------
  pinMode(buttons,INPUT);
  pinMode(statusLED,OUTPUT);
  pinMode(errorLED,OUTPUT);
  
  //----- Sensors --------------------------------------------------------------
  
  //----- Display --------------------------------------------------------------
  oled.setI2CAddress(oledDisplay);
  oled.begin();
  oled.setFont(u8g2_font_6x10_mf); //set font w5 h10
  maxpages = arp - 1; //maximum number of pages + NTP?
  
  //----- Real Time Clock ------------------------------------------------------
  setSyncProvider(getTeensy3Time);
  
  //----- Ethernet -------------------------------------------------------------
  //fetch mac address
  Serial.println("Fetching mac address...");
  OLEDprint(0,0,1,1,">>> Ethernet <<<");
  uint8_t mac[6];
  Ethernet.macAddress(mac);  //This is informative; it retrieves, not sets
  char dataString[18];
  sprintf(dataString,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  //print to serial
  Serial.print("MAC: "); 
  Serial.println(dataString);
  //print to oled
  OLEDprint(1,0,0,1,"MAC");
  OLEDprint(1,4,0,1,dataString);

  //Start ethernet with DHCP
  Serial.println("Starting Ethernet with DHCP...");
  OLEDprint(2,0,0,1,"get IP via DHCP...");
  uint8_t Ethstate = 0;
  while(Ethstate == 0){
    if(!Ethernet.begin()){  //Starting Ethernet
      Serial.printf("Failed to start Ethernet\r\n");
      
      OLEDprint(4,0,0,0,"Failed to start Eth.");
      OLEDprint(5,0,0,0,"CONFIRM");
      OLEDprint(5,14,0,1,"REPEAT");
      uint8_t buttonpress = getButton();
      if(buttonpress == 1) Ethstate = 1;
      OLEDprint(4,0,0,0,"                    "); //clear line
      OLEDprint(5,0,0,1,"                    ");
    }
    if(!Ethernet.waitForLocalIP(DHCPTimeout)){ //15 second timeout to get IP address via DHCP
      Serial.printf("Failed to get IP address from DHCP\r\n");
      OLEDprint(4,0,0,1,"Failed to get IP");
      OLEDprint(5,0,0,1,"CONFIRM");
      OLEDprint(5,14,0,1,"REPEAT");
      uint8_t buttonpress = getButton();
      if(buttonpress == 1) Ethstate = 1;
      OLEDprint(4,0,0,0,"                    "); //clear line
      OLEDprint(5,0,0,1,"                    ");
    }
    else{
      Ethstate = 1;
    }
  }

  //print to display
  IPAddress ip = Ethernet.localIP();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("IP:          ");
  Serial.println(dataString);
  OLEDprint(2,0,0,0,"                    ");
  OLEDprint(2,0,0,0,"IP");
  OLEDprint(2,4,0,0,dataString);

  ip = Ethernet.subnetMask();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("Subnet Mask: ");
  Serial.println(dataString);
  OLEDprint(3,0,0,0,"Sub");
  OLEDprint(3,4,0,0,dataString);

  ip = Ethernet.gatewayIP();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("Gateway:     ");
  Serial.println(dataString);
  OLEDprint(4,0,0,0,"Gat");
  OLEDprint(4,4,0,0,dataString);

  ip = Ethernet.dnsServerIP();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("DNS:         ");
  Serial.println(dataString);
  OLEDprint(5,0,0,0,"DNS");
  OLEDprint(5,4,0,1,dataString);

  // Start UDP listening on the NTP port
  udp.begin(NTPPort);

  //create NTP request package
  memset(ntpbuf, 0, 48);
  ntpbuf[0] = 0b00'100'011; // LI leap indicator warns of leap seconds, Version number (current 3), Mode 3 = client
  ntpbuf[1] = 0;    //stratum, 0 = unspecified
  ntpbuf[2] = 6;    //maximum time between successive messages 2^x
  ntpbuf[3] = 0xF1; // Peer Clock Precision -15

  ntpbuf[12] = 90;  //reference identifier "x" for experimental
  ntpbuf[13] = 90;
  ntpbuf[14] = 90;
  ntpbuf[15] = 90;
  
  //----- Setup RFID readers ---------------------------------------------------
  //measure resonant frequency and confirm/repeat on detune
  
  while(1){
  
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
  
  // enableReader(RFIDreader[0][0]);
  // enableReader(RFIDreader[0][1]);
  
  // disableReader(RFIDreader[1][0]);
  // disableReader(RFIDreader[1][1]);
  // disableReader(RFIDreader[2][0]);
  // disableReader(RFIDreader[2][1]);

} //end of setup

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){

  //create/clear strings that get written to uSD card
  String RFIDdataString = "";   //holds tag and date
  String SENSORDataString = ""; //holds various sensor and diagnostics data
  String MISCdataString = "";   //holds info of time sync events (and possibly other events)
  
  //----------------------------------------------------------------------------
  //record RFID tags -----------------------------------------------------------
  //----------------------------------------------------------------------------
  //>=80ms are required to cold-start a tag for a successful read (at reduced range)
  //>=90ms for full range, increasing further only seems to increase range due to noise
  //rather than requirements of the tag and coil for energizing (100ms is chosen as a compromise)
  
  if(globalRFIDtime >= 100){
    globalRFIDtime = 0;  //reset time
    
    if(globalRFIDtoggle == 1){
      for(uint8_t r = 0;r < arp;r++){
        globalRFIDtoggle = 0; //toggle the toggle
        switchReaders(RFIDreader[r][1],RFIDreader[r][0]); //enable reader2, disable reader1
        
        uint8_t tag_status = fetchtag(RFIDreader[r][0],1); //fetch data reader1 collected during on-time saved in variable: tag
        for(uint8_t i = 0; i < sizeof(tag); i++) currenttag1[r][i] = tag[i]; //copy received tag to current tag
        
        //compare current and last tag 0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag_switch = compareTags(currenttag1[r],lasttag1[r]);
        char reader[4] = {'R',RFIDreaderNames[r],'1'};
        RFIDdataString = createRFIDDataString(currenttag1[r], lasttag1[r], tag_switch, reader, RFIDdataString); //create datastring that is written to uSD
        for(uint8_t i = 0; i < sizeof(currenttag1[r]); i++) lasttag1[r][i] = currenttag1[r][i]; //copy currenttag to lasttag
        
        if(tag_status == 1){ //tag is not empty
          for(uint8_t i = 0; i < sizeof(currenttag1[r]); i++) latestreadtag1[r][i] = currenttag1[r][i]; //copy latesttag
          latest_tagtime1[r] = Teensy3Clock.get();
        }
        
      }
    }
    else{
      for(uint8_t r = 0;r < arp;r++){
        globalRFIDtoggle = 1; //toggle the toggle
        switchReaders(RFIDreader[r][0],RFIDreader[r][1]); //enable reader1, disable reader2
        
        uint8_t tag_status = fetchtag(RFIDreader[r][1],1); //fetch data reader2 collected during on-time saved in variable: tag
        for(uint8_t i = 0; i < sizeof(tag); i++) currenttag2[r][i] = tag[i]; //copy received tag to current tag
        
        //compare current and last tag 0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag_switch = compareTags(currenttag2[r],lasttag2[r]);
        char reader[4] = {'R',RFIDreaderNames[r],'2'};
        RFIDdataString = createRFIDDataString(currenttag2[r], lasttag2[r], tag_switch, reader, RFIDdataString); //create datastring that is written to uSD
        for(uint8_t i = 0; i < sizeof(currenttag2[r]); i++) lasttag2[r][i] = currenttag2[r][i]; //copy currenttag to lasttag
        
        if(tag_status == 1){ //tag is not empty
          for(uint8_t i = 0; i < sizeof(currenttag2[r]); i++) latestreadtag2[r][i] = currenttag2[r][i]; //copy latesttag
          latest_tagtime2[r] = Teensy3Clock.get();
        }
      }
    }
  }
  
  //----------------------------------------------------------------------------
  //update RTC -----------------------------------------------------------------
  //----------------------------------------------------------------------------
  if((globalRFIDtime < 25) && //only sync if last RFID sync was 25ms ago so we are not blocking the switching
    (((((Teensy3Clock.get() % syncinterval) == 0) && (NTPsynctime > 2000)) || //sync at full x minutes/seconds but last sync must be at least 2 seconds ago
    (NTPsynctime > 1000 * (syncinterval + 30))) || //or if we miss the exact second for syncing, sync after x time has elapsed
    (force_sync && (NTPsynctime > 1000)))){ //or if forced sync due to sync failure
    
    NTPsynctime = 0;   //reset time of last sync
    char timeinfo[38]; //for verbose NTP server responses
    
    //perform NTP sync - if first sync after failed, don't add to median array
    uint8_t NTPstate;
    if(NTP_sync_failed == 0){ //if it didn't fail previously, do "normal" sync
      NTPstate = NTPsync(1,1,1,1);
    }
    else{ //if previously failed, don't add drift to median
      NTPstate = NTPsync(1,0,1,1);
    }
    
    if(NTPstate == 0){ //--- NTP sync successful
      NTP_sync_failed = 0; //reset failure counter
      force_sync = 0;      //disable force sync
      
      //add detailed NTP data to log ~0.1ms
      MISCdataString = createMISCDataString("NTP",vhrTime("loc send: ",NTP_timestamps[0]),"",MISCdataString);
      MISCdataString = createMISCDataString("NTP",vhrTime("srv rec.: ",NTP_timestamps[1]),"",MISCdataString);
      MISCdataString = createMISCDataString("NTP",vhrTime("srv send: ",NTP_timestamps[2]),"",MISCdataString);
      MISCdataString = createMISCDataString("NTP",vhrTime("loc rec.: ",NTP_timestamps[3]),"",MISCdataString);
      MISCdataString = createMISCDataString("NTP",vhrTime("loc adj.: ",NTP_timestamps[4]),"",MISCdataString);

      MISCdataString = createMISCDataString("NTP","RTC diff from NTP ms",RTC_drift_ms,MISCdataString);
      MISCdataString = createMISCDataString("NTP","server response ms",server_res_ms,MISCdataString);
      if(median_ok){  //only log ppm drift if we have collected a full arrays worth of drift data
        MISCdataString = createMISCDataString("NTP","ppm",RTC_drift_median_mean * 1000000,MISCdataString);
      }
    }
    else{ //--- if NTPsync was unsuccessful, use drift data collected from previous syncs to correct RTC
      NTP_sync_failed++; //increase sync fail counter
      MISCdataString = createMISCDataString("NTP","Sync failed state",NTPstate,MISCdataString);
      
      if(NTP_sync_failed >= 3){ //allow up to 3 failed online sync attempts before using saved drift values
        force_sync = 0;         //disable force sync when switching to offline sync
        
        MISCdataString = createMISCDataString("NTP","offline sync","",MISCdataString);
        
        uint8_t syncstatus = NTPsync(1,0,0,0); //perform offline RTC sync

        if(syncstatus == 99) MISCdataString = createMISCDataString("NTP","too few drift samples","",MISCdataString);
        else{
          MISCdataString = createMISCDataString("NTP",vhrTime("loc now.: ",NTP_timestamps[3]),"",MISCdataString);
          MISCdataString = createMISCDataString("NTP",vhrTime("loc dis.: ",NTP_timestamps[4]),"",MISCdataString);
          MISCdataString = createMISCDataString("NTP","RTC adj by ms",(NTP_timestamps[4] - NTP_timestamps[3]) * 1000,MISCdataString);
        }
      }
      else{ //try sync again
        force_sync = 1;
      }
    }
  }
  
  //----------------------------------------------------------------------------
  //update display -------------------------------------------------------------
  //----------------------------------------------------------------------------
  if((globalRFIDtime < 50) && (displaytime > 1000)){ //once every second, and only if we still have 50ms to go before next sync
    displaytime = 0;
    uint8_t button = getNBButton();
    
    //switch display on/off if button pressed
    if(button ==  2){
      displayon = !displayon;
      if(!displayon){
        oled.clearBuffer();   //clear display
        oled.sendBuffer();
      }
    }
    
    if(displayon){
      oled.clearBuffer(); //clear display
      
      time_t rtctime = Teensy3Clock.get(); //get current time
      char ndate[11]; //DD-MM-YYYY
      sprintf(ndate,"%02u-%02u-%04u",day(rtctime),month(rtctime),year(rtctime));
      
      ////--- draw UI elements ---
      if(button == 1) page -= 1;
      if(button == 3) page += 1;
      if(page > maxpages) page = 0;
      if(page < 0) page = maxpages;
      
      OLEDprint(0,0,0,0,nicetime(rtctime));
      OLEDprint(0,11,0,0,ndate);
      
      OLEDprint(5,0,0,0,"PREV");
      OLEDprint(5,17,0,0,"NEXT");
      OLEDprint(5,9,0,0,"OFF");
      
      //--- RFID pages display last read tag
      uint8_t r;
      if(page == 0) r = 0;
      if(page == 1) r = 1;
      if(page == 2) r = 2;
      
      char reader[3] = {'R',RFIDreaderNames[r]}; //create string for the reader name
      //reader 1
      String shorttagID = getID(latestreadtag1[r]); //get tag in string format
      shorttagID = shorttagID.substring(shorttagID.length()-7,shorttagID.length()+1); //use last 7 digits of RFID tag
      float tagTemp = getTemperatureC(latestreadtag1[r]); //get temperature in °C format
      String hrtag;
      hrtag = String(reader) + "1: " + shorttagID + " | " + String(tagTemp,1) + "C";  //make a nice string for printing
      String hrtagtime = String(reader) + "1: " + nicetime(latest_tagtime1[r]); //make a nice string for printing
      
      OLEDprint(1,0,0,0,hrtag);
      OLEDprint(2,0,0,0,hrtagtime);
      //reader 2
      shorttagID = getID(latestreadtag2[r]);
      shorttagID = shorttagID.substring(shorttagID.length()-7,shorttagID.length()+1);
      tagTemp = getTemperatureC(latestreadtag2[r]);
      hrtag = String(reader) + "2: " + shorttagID + " | " + String(tagTemp,1) + "C";
      hrtagtime = String(reader) + "2: "  + nicetime(latest_tagtime2[r]);
      
      OLEDprint(3,0,0,0,hrtag);
      OLEDprint(4,0,0,0,hrtagtime);
      
      //--- update display ---
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
  //log MISC events ~2-3ms
  if(MISCdataString.length() != 0){
    dataFile.println(MISCdataString);	//append Datastring to file
		dataFile.flush();
		dataFileBackup.println(MISCdataString);
		dataFileBackup.flush();
    Serial.println(MISCdataString);
    Serial.println("");
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
  char ntime[11]; //HH:MM:SS
  sprintf(ntime,"%02u:%02u:%02u",hour(nowtime),minute(nowtime),second(nowtime));
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
  Wire.endTransmission(1);
  delayMicroseconds(1200); //reduces down-time of antennas since startup isn't instant
  //turn off the other
  Wire.beginTransmission(readerOFF);
  Wire.write(0);
  Wire.endTransmission(1);
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

//sum all values from tag
int16_t tagSum(byte tag[]){
  int16_t sum = 0;
  for(uint8_t i = 0; i < sizeof(tag); i++) sum = sum + tag[i];
  return sum;
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
  time_t nowtime = Teensy3Clock.get();
  
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

//critical error, flash LED, stop everything -----------------------------------
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

//returns which button is currently pressed (non-blocking) ---------------------
uint8_t getNBButton(){
  int16_t input = analogRead(buttons);
  
  if(input <= 150) return 1;
  if(input > 150 && input <= 450) return 2;
  if(input > 450 && input <= 850) return 3;
}

//read fractional seconds from RTC 1/32768 (2^15) sec. -------------------------
int16_t readRTCfrac(){
  uint32_t hi1, lo1, hi2, lo2;
  hi1 = SNVS_HPRTCMR;
  lo1 = SNVS_HPRTCLR;
  while(1){
    hi2 = SNVS_HPRTCMR;
    lo2 = SNVS_HPRTCLR;
    if (lo1 == lo2 && hi1 == hi2){
      return (int16_t)(lo2 & 0x7fff); //return last 15 bits
    }
    hi1 = hi2;
    lo1 = lo2;
  }
}

//sets RTC to seconds and fractions of seconds ---------------------------------
void rtc_set_secs_and_frac(uint32_t secs, uint32_t frac){
	// stop the RTC
	SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
	while (SNVS_HPCR & SNVS_HPCR_RTC_EN); // wait
	// stop the SRTC
	SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
	while (SNVS_LPCR & SNVS_LPCR_SRTC_ENV); // wait
	// set the SRTC
  SNVS_LPSRTCLR = ((secs & 0x1ffffUL) << 15) | (frac & 0x7fff);
	SNVS_LPSRTCMR = secs >> 17;
	// start the SRTC
	SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
	while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)); // wait
	// start the RTC and sync it to the SRTC
	SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;
}

//take unixtime and fractions of seconds 2**15 and convert to double -----------
double doubleTime15(uint32_t seconds, uint32_t frac15){
  return seconds + ((double)frac15/32768);
}

//take unixtime and fractions of seconds 2**32 and convert to double -----------
double doubleTime32(uint32_t seconds, uint32_t frac32){
  return seconds + ((double)frac32/UINT32_MAX);
}

//take double time (unixtime with fractions) and return uint unixtime and fractions separately as fractions 2**15
void fracTime15(double dtime, uint32_t *seconds, uint32_t *frac15){
  double seconds_temp; //seconds
  double frac;    //fractions of seconds
  
  frac = modf(dtime, &seconds_temp); //split into integer/fractional parts -----
  *frac15 = frac * 32768;  //convert to 2**15 fractions
  *seconds = seconds_temp; //
}

//Fetch NTP time and update RTC ~3ms dependent on server response time ---------
uint8_t NTPsync(bool update_time, bool save_drift, bool burst, bool online_sync){
  double dt0, dt1, dt2, dt3; //the four timepoints of the NTP package
  double dtheta; //difference of local clock to server clock
  double timeout_buf_us; //let NTP request time out if server not available
  double newtime, dis_now_time; //new time the RTC should be set to
  
  if(online_sync){
    int8_t repeat = 3; //to perform multiple syncs in quick succession
    if(!burst) repeat = 1;
    
    while(repeat > 0){
      //Set the Transmit Timestamp < 0.001 ms
      while(udp.parsePacket() > 0); //clear any udp data left in the buffer
      
      //nothing slow from here until time is written!
      uint32_t send_lt = Teensy3Clock.get();  //get local time to send
      uint32_t send_lt_frac15 = readRTCfrac(); //fractions of seconds 0 - 2^15
      if(send_lt >= EBreakTime) send_lt -= EBreakTime;  //see epochs etc.
      else send_lt += EpochDiff;
      
      //--- Send the packet. Dependent on server response time, with local network fritzbox this can take about ~8 ms (avg ~3 ms)
      if(!Ethernet.linkState()) return 5; //check if we have an ethernet connection
      if(!udp.send(NTPserver,NTPPort,ntpbuf,48)) return 1; //server address, port, data, length, this takes seconds to timeout
      
      elapsedMicros timeout_us;  //micros for benchmarking
      while((udp.parsePacket() < 0) && (timeout_us < NTP_server_timeout));   //returns size of packet or <= 0 if no packet, timeout
      if(timeout_us >= NTP_server_timeout) return 2; //check if the receiving timed out

      timeout_buf_us = timeout_us; //measure how long it took for the server to answer
      
      const uint8_t *ntpbuf = udp.data(); //returns pointer to received package data
      
      //seconds and fractions of local clock when NTP packet is received
      uint32_t receive_lt = Teensy3Clock.get();
      uint32_t receive_lt_frac15 = readRTCfrac();
      
      //check if the data we received is according to spec < 0.001 ms
      int mode = ntpbuf[0] & 0x07;
      if(((ntpbuf[0] & 0xc0) == 0xc0) || //LI == 3 (Alarm condition)
        (ntpbuf[1] == 0) ||              //Stratum == 0 (Kiss-o'-Death)
        !(mode == 4 || mode == 5)) {     //Must be Server or Broadcast mode
        return 3;
      }
      
      //seconds and fractions when NTP received request < 0.002 ms until adjusting time
      uint32_t receive_st        = (uint32_t{ntpbuf[32]} << 24) | (uint32_t{ntpbuf[33]} << 16) | (uint32_t{ntpbuf[34]} << 8) | uint32_t{ntpbuf[35]}; //receive server time
      uint32_t receive_st_frac32 = (uint32_t{ntpbuf[36]} << 24) | (uint32_t{ntpbuf[37]} << 16) | (uint32_t{ntpbuf[38]} << 8) | uint32_t{ntpbuf[39]};
      //seconds and fractions when NTP sent
      uint32_t send_st        = (uint32_t{ntpbuf[40]} << 24) | (uint32_t{ntpbuf[41]} << 16) | (uint32_t{ntpbuf[42]} << 8) | uint32_t{ntpbuf[43]};
      uint32_t send_st_frac32 = (uint32_t{ntpbuf[44]} << 24) | (uint32_t{ntpbuf[45]} << 16) | (uint32_t{ntpbuf[46]} << 8) | uint32_t{ntpbuf[47]};
      
      //Discard if reply empty, also discard when the transmit timestamp is zero
      if(send_st == 0) return 4;
      
      //See "NTP Timestamp Format"
      if((send_lt & 0x80000000U) == 0) send_lt += EBreakTime;
      else send_lt -= EpochDiff;
      if((send_st & 0x80000000U) == 0) send_st += EBreakTime;
      else send_st -= EpochDiff;
      if((receive_st & 0x80000000U) == 0) receive_st += EBreakTime;
      else receive_st -= EpochDiff;
      
      //Just convert everything to double, avoides all rollover headaches, still fast/accurate enough
      dt0 = doubleTime15(send_lt,send_lt_frac15);
      dt1 = doubleTime32(receive_st,receive_st_frac32);
      dt2 = doubleTime32(send_st,send_st_frac32);
      dt3 = doubleTime15(receive_lt,receive_lt_frac15);
      dtheta = (dt1 - dt0 + dt2 - dt3) / 2; //offset of local time vs NTP server time
      
      burst_array[repeat-1] = dtheta;
      
      repeat -= 1;
    }
  
    if(burst) dtheta = median3(burst_array[0], burst_array[1], burst_array[2]); //get the median offset value of the last three measurements
  
    newtime = dt3 + dtheta + clock_set_offset; //seconds + millis RTC should be set to (receive_local_time + offset)
  
    uint32_t setseconds, setfrac15;
    fracTime15(newtime, &setseconds, &setfrac15);
  
    //Set the RTC and time ~0.939 ms
    if(update_time) rtc_set_secs_and_frac(setseconds,setfrac15); //set time and adjust for transmit delay, frac will only use lower 15bits
    
    //read back time from adjusted RTC (for debugging)
    //uint32_t adjust_lt = Teensy3Clock.get();
    //uint32_t adjust_lt_frac15 = readRTCfrac();
    
    //pass time for printing
    NTP_timestamps[0] = dt0;     //send local time
    NTP_timestamps[1] = dt1;     //receive server time
    NTP_timestamps[2] = dt2;     //send server time
    NTP_timestamps[3] = dt3;     //receive local time
    NTP_timestamps[4] = newtime; //adjusted local time
    
    RTC_drift_ms = dtheta * 1000;  //difference between local RTC and NTP server
    server_res_ms = timeout_buf_us / 1000; //time between sending UDP package and receiving
  }
  
  //if we want to perform an offline sync based on measured ppm
  if(!online_sync){
    if(median_ok){ //allow offline RTC adjust only if we have collected enough data to reliably correct the time
      double now_time = doubleTime15(Teensy3Clock.get(),readRTCfrac()); //get current time
      double timediff = now_time - last_sync;   //time between now and last sync
      dis_now_time = now_time + (RTC_drift_median_mean * timediff) + clock_set_offset; //the disciplined now time is the current RTC time adjusted by calculated drift
      
      //split disciplined time to sec+frac15
      uint32_t disseconds,dis_frac15;
      fracTime15(dis_now_time, &disseconds, &dis_frac15);
      
      //update RTC clock with offset determined by calculated drift
      if(update_time) rtc_set_secs_and_frac(disseconds,dis_frac15); //set time and adjust for transmit delay, frac will only use lower 15bits
      
      //pass time for printing
      NTP_timestamps[3] = now_time;     //receive local time
      NTP_timestamps[4] = dis_now_time; //adjusted local time
    }
    else{
      return 99; //if we don't have enough data to correct the time
    }
  }
  
  //when actually syncing, store the adjusted time when the sync was done (~0.014ms with size 31 drift array when saving drift)
  if(update_time){
    second_last_sync = last_sync;
    if(online_sync) last_sync = newtime;
    if(!online_sync) last_sync = dis_now_time;
    
    if(save_drift && online_sync){ //don't save drift when syncing offline
      double timebase_drift = last_sync - second_last_sync;
      double drift_s_per_s = dtheta / timebase_drift;
      
      RTC_drift_array[RTC_drift_it] = drift_s_per_s; //add drift value of current sync to drift array
      
      RTC_drift_it++;
      if(RTC_drift_it >= drift_array_size){
        RTC_drift_it = 0;
        median_ok = 1;  //array was filled at least once
      }
      //sort drift array and get median
      double sort_drift[drift_array_size]; //create new array that will be sorted
      memcpy(sort_drift,RTC_drift_array,sizeof(RTC_drift_array[0])*drift_array_size); //copy array
      qsort(sort_drift, drift_array_size, sizeof(sort_drift[0]), cmpfunc); //sort new array
      RTC_drift_ms_median = sort_drift[(drift_array_size-1)/2]; //get median value from sorted array
      
      //get the "middle" n% of median values and calculate average from them to further improve accuracy with a limited number of median values
      RTC_drift_median_mean = 0;
      uint16_t lower_median = ((drift_array_size-1)/2) - floor(drift_array_size * 0.4); //0.4 = 80%
      uint16_t upper_median = ((drift_array_size-1)/2) + floor(drift_array_size * 0.4);
      for(int m = lower_median;m < upper_median;m++){
        RTC_drift_median_mean += sort_drift[m];
      }
      RTC_drift_median_mean = RTC_drift_median_mean / (upper_median - lower_median);
    }
  }
  return 0;
}

//compare function for getting median ------------------------------------------
int cmpfunc(const void *a, const void *b){
  if(*(double*)a > *(double*)b) return 1;
  else if(*(double*)a < *(double*)b) return -1;
  else return 0;
}

//return median value of three doubles -----------------------------------------
double median3(double a, double b, double c){
    if ((a <= b) && (b <= c)) return b;  // a b c
    if ((a <= c) && (c <= b)) return c;  // a c b
    if ((b <= a) && (a <= c)) return a;  // b a c
    if ((b <= c) && (c <= a)) return c;  // b c a
    if ((c <= a) && (a <= b)) return a;  // c a b
    return b;                            // c b a
}

//Create misc data String ------------------------------------------------------
String createMISCDataString(String identifier, String event1,String event2,String dataString){
  time_t unixtime = Teensy3Clock.get();

  if(dataString != 0) dataString += "\n"; //if datastring is not empty, add newline
  dataString += identifier;
  dataString += ",";
  dataString += unixtime;
  dataString += ",";
  dataString += millis();
  dataString += ",";
  dataString += event1;
  dataString += ",";
  dataString += event2;
  
  return dataString;
}

//create verbose human readable time String textYYYY-MM-DD HH:MM:SS-mss.uss ----
String vhrTime(String text,double time){
  char timeinfo[28];
  
  tmElements_t xt;
  breakTime(time,xt);
  double xt_ms = (time - floor(time)) * 1000;
  
  sprintf(timeinfo,"%04u-%02u-%02u %02u:%02u:%02u-%07.3f",xt.Year + 1970,xt.Month,xt.Day,xt.Hour,xt.Minute,xt.Second,xt_ms);
  
  text += String(timeinfo);
  
  return text;
}

// void test(){
//   if(globalRFIDtoggle == 1){ //switch all readers "simultaniously"
//     globalRFIDtoggle = 0; //toggle the toggle
//     for(uint8_t r = 0;r < arp;r++){
//       switchReaders(RFIDreader[r][1],RFIDreader[r][0]); //enable reader2, disable reader1
      
//       uint8_t tag_status = fetchtag(RFIDreader[r][0],1); //fetch data reader1 collected during on-time saved in variable: tag
//       for(uint8_t i = 0; i < sizeof(tag); i++) currenttag1[r][i] = tag[i]; //copy received tag to current tag
      
//       //compare current and last tag 0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
//       uint8_t tag_switch = compareTags(currenttag1[r],lasttag1[r]);
//       char reader[4] = {'R',RFIDreaderNames[r],'1'};
//       RFIDdataString = createRFIDDataString(currenttag1[r], lasttag1[r], tag_switch, reader, RFIDdataString); //create datastring that is written to uSD
//       for(uint8_t i = 0; i < sizeof(currenttag1[r]); i++) lasttag1[r][i] = currenttag1[r][i]; //copy currenttag to lasttag
//     }
//   }
//   else{
//     globalRFIDtoggle = 1; //toggle the toggle
//     for(uint8_t r = 0;r < arp;r++){ //switch all readers "simultaniously"
//       switchReaders(RFIDreader[r][0],RFIDreader[r][1]); //enable reader1, disable reader2
      
//       uint8_t tag_status = fetchtag(RFIDreader[r][1],1); //fetch data reader2 collected during on-time saved in variable: tag
//       for(uint8_t i = 0; i < sizeof(tag); i++) currenttag2[r][i] = tag[i]; //copy received tag to current tag
      
//       //compare current and last tag 0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
//       uint8_t tag_switch = compareTags(currenttag2[r],lasttag2[r]);
//       char reader[4] = {'R',RFIDreaderNames[r],'2'};
//       RFIDdataString = createRFIDDataString(currenttag2[r], lasttag2[r], tag_switch, reader, RFIDdataString); //create datastring that is written to uSD
//       for(uint8_t i = 0; i < sizeof(currenttag2[r]); i++) lasttag2[r][i] = currenttag2[r][i]; //copy currenttag to lasttag
//     }
//   }
// }
  
  