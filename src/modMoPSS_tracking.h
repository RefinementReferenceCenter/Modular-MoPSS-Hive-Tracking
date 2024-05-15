#ifndef MODMOPSS_TRACKING_H
#define MODMOPSS_TRACKING_H

#include <stdint.h>
#include "Arduino.h"
#include "functions.h"
#include <TimeLib.h>         //Manage Real Time CLock
#include <i2c_driver_wire.h> //I2C communication !!! libraries using Wire.h must be adjusted to use i2c_driver_wire.h instead !!!
#include <SdFat.h>           //Access SD Cards
#include <U8g2lib.h>         //for SSD1306 OLED Display
#include <QNEthernet.h>      //for ethernet
#include <ModMoPSS_logo.h>
#include <ansi.h>

bool startChecks();
bool checkModule(uint8_t address);
String vhrTime(String text,double time);
uint8_t NTPsync(bool update_time, bool save_drift, bool burst, bool online_sync);


uint8_t getNBButton();
uint8_t getButton();
void confirm();
void OLEDprint();

time_t getTeensy3Time();

void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, String text);
void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, int32_t number);
void OLEDprintFraction(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, float number, uint8_t decimals);
void criticalerror();
void criticalerrorMessage(char *message,char *message2);


void enableReader(uint8_t reader);
void disableReader(uint8_t reader);
uint8_t compareTags(byte currenttag[], byte lasttag[]);
int16_t tagSum(byte tag[]);
uint8_t fetchtag(byte reader, byte busrelease);
void switchReaders(byte readerON, byte readerOFF);
void setReaderMode(uint8_t reader,uint8_t mode);
uint32_t fetchResFreq(uint8_t reader);
uint32_t fetchResFreqCont(uint8_t reader);

#endif