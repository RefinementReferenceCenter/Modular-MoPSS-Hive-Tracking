#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <stdint.h>
#include "Arduino.h"
#include <TimeLib.h>         //Manage Real Time CLock
double median3(double a, double b, double c);
String nicetime(time_t nowtime);
String getID(uint8_t in[7]);
uint16_t getCountryCode(uint8_t in[7]);
uint8_t getTemperature(uint8_t in[7]);
float getTemperatureC(uint8_t in[7]);
int cmpfunc(const void *a, const void *b);
String vhrTime(String text,double time);

String createRFIDDataString(byte currenttag[], byte lasttag[], int tagchange, char identifier[], String dataString);
String createMISCDataString(String identifier, String event1,String event2,String dataString);
String createSENSORDataString(String identifier, String event, String dataString);



double doubleTime15(uint32_t seconds, uint32_t frac15);
double doubleTime32(uint32_t seconds, uint32_t frac32);
void fracTime15(double dtime, uint32_t *seconds, uint32_t *frac15);

int16_t tagSum(byte tag[]);
uint8_t compareTags(byte currenttag[], byte lasttag[]);
#endif // !FUNCTIONS_H
