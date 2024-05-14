
#include "functions.h"
//return median value of three doubles -----------------------------------------
double median3(double a, double b, double c){
    if ((a <= b) && (b <= c)) return b;  // a b c
    if ((a <= c) && (c <= b)) return c;  // a c b
    if ((b <= a) && (a <= c)) return a;  // b a c
    if ((b <= c) && (c <= a)) return c;  // b c a
    if ((c <= a) && (a <= b)) return a;  // c a b
    return b;                            // c b a
}
//Return time as string in HH:MM:SS format -------------------------------------
String nicetime(time_t nowtime){
  char ntime[11]; //HH:MM:SS
  sprintf(ntime,"%02u:%02u:%02u",hour(nowtime),minute(nowtime),second(nowtime));
	return ntime;
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

//compare function for getting median ------------------------------------------
int cmpfunc(const void *a, const void *b){
  if(*(double*)a > *(double*)b) return 1;
  else if(*(double*)a < *(double*)b) return -1;
  else return 0;
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