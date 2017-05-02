//Rx5808Fns.cpp:  Code from rx5808-pro-diversity project
//                https://github.com/sheaivey/rx5808-pro-diversity
//
//  4/29/2017 -- [ET]  Code modified for ArduVidRx usage.
//

/*
 * SPI driver based on fs_skyrf_58g-main.c Written by Simon Chambers
 * TVOUT by Myles Metzel
 * Scanner by Johan Hermen
 * Inital 2 Button version by Peter (pete1990)
 * Refactored and GUI reworked by Marko Hoepken
 * Universal version my Marko Hoepken
 * Diversity Receiver Mode and GUI improvements by Shea Ivey
 * OLED Version by Shea Ivey
 * Seperating display concerns by Shea Ivey

The MIT License (MIT)

Copyright (c) 2015 Marko Hoepken

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Config.h"
#include "Rx5808Fns.h"

// Channels to send to the SPI registers
const uint16_t channelRegTable[] PROGMEM = {
  // Channel 1 - 8
  0x2A05,    0x299B,    0x2991,    0x2987,    0x291D,    0x2913,    0x2909,    0x289F,    // Band A
  0x2903,    0x290C,    0x2916,    0x291F,    0x2989,    0x2992,    0x299C,    0x2A05,    // Band B
  0x2895,    0x288B,    0x2881,    0x2817,    0x2A0F,    0x2A19,    0x2A83,    0x2A8D,    // Band E
  0x2906,    0x2910,    0x291A,    0x2984,    0x298E,    0x2998,    0x2A02,    0x2A0C,    // Band F / Airwave
#if USE_LBAND_FLAG
  0x281D,    0x288F,    0x2902,    0x2914,    0x2987,    0x2999,    0x2A0C,    0x2A1E,    // Band C / Immersion Raceband
  0x2609,    0x261C,    0x268E,    0x2701,    0x2713,    0x2786,    0x2798,    0x280B     // Band D / 5.3
#else
  0x281D,    0x288F,    0x2902,    0x2914,    0x2987,    0x2999,    0x2A0C,    0x2A1E     // Band C / Immersion Raceband
#endif
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] PROGMEM = {
  // Channel 1 - 8
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F / Airwave
#if USE_LBAND_FLAG
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // Band C / Immersion Raceband
  5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621  // Band D / 5.3
#else
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Band C / Immersion Raceband
#endif
};

// do coding as simple hex value to save memory.
//const uint8_t channelNames[] PROGMEM = {
//  0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, // Band A
//  0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, // Band B
//  0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, // Band E
//  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, // Band F / Airwave
//#if USE_LBAND_FLAG
//  0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, // Band C / Immersion Raceband
//  0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8  // BAND D / 5.3
//#else
//  0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8  // Band C / Immersion Raceband
//#endif
//};

// All Channels of the above List ordered by Mhz
const uint8_t channelSortTable[] PROGMEM = {
#if USE_LBAND_FLAG
  40, 41, 42, 43, 44, 45, 46, 47, 19, 32, 18, 17, 33, 16, 7, 34, 8, 24, 6, 9, 25, 5, 35, 10, 26, 4, 11, 27, 3, 36, 12, 28, 2, 13, 29, 37, 1, 14, 30, 0, 15, 31, 38, 20, 21, 39, 22, 23
#else
  19, 32, 18, 17, 33, 16, 7, 34, 8, 24, 6, 9, 25, 5, 35, 10, 26, 4, 11, 27, 3, 36, 12, 28, 2, 13, 29, 37, 1, 14, 30, 0, 15, 31, 38, 20, 21, 39, 22, 23
#endif
};

     //array of band codes in 'channelFreqTable[]' order:
#if USE_LBAND_FLAG
const char freqBandCodesArray[] = "ABEFRL";
#else
const char freqBandCodesArray[] = "ABEFR";
#endif
#define NUM_FREQBAND_CODES ((CHANNEL_MAX_INDEX+1)/CHANNEL_BAND_SIZE)

uint8_t rx5808RssiInPin = RSSI_PRI_PIN;
uint8_t rx5808MinTuneTimeMs = RX5808_MIN_TUNETIME;
uint8_t lastChannelIndex = 0;
unsigned long timeOfLastTune = 0;      //time of last tuner-channel change
uint16_t rx5808RawRssiMin = DEF_RAWRSSI_MIN;
uint16_t rx5808RawRssiMax = DEF_RAWRSSI_MAX;
//uint16_t rssi_setup_min_a=RAW_RSSI_MIN;
//uint16_t rssi_setup_max_a=RAW_RSSI_MAX;


/*###########################################################################*/
/*******************/
/*   SUB ROUTINES  */
/*******************/

//uint8_t channel_from_index(uint8_t channelIndex)
//{
//  uint8_t loop=0;
//  uint8_t channel=0;
//  for (loop=0; loop<=CHANNEL_MAX_INDEX; loop++)
//  {
//    if(pgm_read_byte_near(channelSortTable + loop) == channelIndex)
//    {
//      channel=loop;
//      break;
//    }
//  }
//  return (channel);
//}

//Returns the value from the channel-frequency table for the given index.
uint16_t getChannelFreqTableEntry(int idx)
{
  return pgm_read_word_near(channelFreqTable + idx);
}

//Returns the value from the channel-register table for the given index.
uint16_t getChannelRegTableEntry(int idx)
{
  return pgm_read_word_near(channelRegTable + idx);
}

//Returns the value from the channel-sorted-indices table for the given index.
uint8_t getChannelSortTableEntry(int idx)
{
  return pgm_read_word_near(channelSortTable + idx);
}

//Returns the 'channelFreqTable[]' index corresponding to the given
// frequency in MHz, or -1 if no match.
int getIdxForFreqInMhz(uint16_t freqVal)
{
  for(int idx=CHANNEL_MIN_INDEX; idx<=CHANNEL_MAX_INDEX; ++idx)
  {
    if(getChannelFreqTableEntry(idx) == freqVal)
      return idx;
  }
  return -1;
}

void waitRssiReady()
{
  // CHECK FOR MINIMUM DELAY
  // check if RSSI is stable after tune by checking the time
  uint16_t tune_time = (uint16_t)(millis() - timeOfLastTune);
  if(tune_time < rx5808MinTuneTimeMs)
  {
      // wait until tune time is full filled
    delay(rx5808MinTuneTimeMs-tune_time);
  }
}

//Reads and averages a set of RSSI samples for the currently-tuned channel.
// Returns:  A raw RSSI value.
uint16_t readRawRssiValue()
{
  int rssiA = 0;

  analogRead(rx5808RssiInPin);              //pre-read to improve I/O
  for (uint8_t i = 0; i < RSSI_READS; i++)
  {
    rssiA += analogRead(rx5808RssiInPin);
  }
  rssiA = rssiA / RSSI_READS; // average of RSSI_READS readings

    // special case for RSSI setup
//    if(state==STATE_RSSI_SETUP)
//    { // RSSI setup
//        if(rssiA < rssi_setup_min_a)
//        {
//            rssi_setup_min_a=rssiA;
//        }
//        if(rssiA > rssi_setup_max_a)
//        {
//            rssi_setup_max_a=rssiA;
//        }
//    }

  return rssiA;
}

//Scales the given raw-RSSI value to be in the MIN_RSSI_VAL to
// MAX_RSSI_VAL range.
uint16_t scaleRawRssiValue(uint16_t rawRssiVal)
{
                   //scale from 0 to 100:
  int val = map(rawRssiVal,rx5808RawRssiMin,rx5808RawRssiMax,
                                                 MIN_RSSI_VAL,MAX_RSSI_VAL);
                   //clip values to only be within range:
  return constrain(val,MIN_RSSI_VAL,MAX_RSSI_VAL);
}

//Reads and returns a single RSSI sample for the currently-tuned channel.
// Returns:  An raw RSSI value.
uint16_t sampleRawRssiValue()
{
  analogRead(rx5808RssiInPin);         //pre-read to improve I/O
  return analogRead(rx5808RssiInPin);
}

void SERIAL_SENDBIT1()
{
  digitalWrite(RX5808_CLK_PIN, LOW);
  delayMicroseconds(1);

  digitalWrite(RX5808_DATA_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(RX5808_CLK_PIN, HIGH);
  delayMicroseconds(1);

  digitalWrite(RX5808_CLK_PIN, LOW);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0()
{
  digitalWrite(RX5808_CLK_PIN, LOW);
  delayMicroseconds(1);

  digitalWrite(RX5808_DATA_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(RX5808_CLK_PIN, HIGH);
  delayMicroseconds(1);

  digitalWrite(RX5808_CLK_PIN, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW()
{
  delayMicroseconds(1);
  digitalWrite(RX5808_SEL_PIN, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH()
{
  delayMicroseconds(1);
  digitalWrite(RX5808_SEL_PIN, HIGH);
  delayMicroseconds(1);
}

void setChannelByRegVal(uint16_t regVal, uint16_t freqInMhz)
{
  uint8_t i;

  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  //delay(2);
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0();

  // remaining zeros
  for (i = 20; i > 0; i--)
    SERIAL_SENDBIT0();

  // Clock the data in
  SERIAL_ENABLE_HIGH();
  //delay(2);
  delayMicroseconds(1);
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)
  {
    // Is bit high or low?
    if (regVal & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    regVal >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  //delay(2);

  digitalWrite(RX5808_SEL_PIN, LOW);
  digitalWrite(RX5808_CLK_PIN, LOW);
  digitalWrite(RX5808_DATA_PIN, LOW);

    //keep time of tune to make sure that RSSI is stable when required
  timeOfLastTune = millis();
}

//void setChannelByIdx(uint8_t idx, uint16_t freqInMhz)
//{
//  setChannelByRegVal(getChannelRegTableEntry(idx),freqInMhz);
//}

// calculate the frequency to bit bang payload
//  https://github.com/sheaivey/rx5808-pro-diversity/issues/75
uint16_t freqMhzToRegVal(uint16_t freqInMhz)
{
  uint16_t tf, N, A;
  tf = (freqInMhz - 479) / 2;
  N = tf / 32;
  A = tf % 32;
  return (N<<7) + A;
}

//Convert register value to frequency in MHz
// FreqMHz = 2*(N*32+A) + 479
uint16_t regValToFreqMhz(uint16_t regVal)
{
  uint16_t N, A;
  N = regVal >> 7;
  A = regVal & 0x3F;
  return 2 * (N*32 + A) + 479;
}

//Set module frequency to given freq value, in MHz.
void setChannelByFreq(uint16_t freqInMhz)
{
  setChannelByRegVal(freqMhzToRegVal(freqInMhz),freqInMhz);
}

//Returns true if the given index is for an L-band frequency.
boolean isLBandChannelIndex(int idx)
{
  return (idx >= LBAND_FIRST_INDEX);
}


/*###########################################################################*/


void rx5808setup()
{
  int freqVal;

  // IO INIT
  // initialize digital pin 13 LED as an output.
//  pinMode(led, OUTPUT); // status pin for TV mode errors
//  digitalWrite(led, HIGH);
  // SPI pins for RX control
  pinMode(RX5808_SEL_PIN, OUTPUT);
  pinMode(RX5808_DATA_PIN, OUTPUT);
  pinMode(RX5808_CLK_PIN, OUTPUT);

  // set the channel as soon as we can
  // faster boot up times :)
//    setChannelModule(channelIndex);
//    last_channel_index = channelIndex;

  // Setup Done - Turn Status LED off.
//  digitalWrite(led, LOW);

#ifdef RSSI_SEC_PIN     //if secondary pin then check if primary has signal
  if(readRawRssiValue() < CHK_RAWRSSI_MIN)
  {
    delay(20);
    if(readRawRssiValue() < CHK_RAWRSSI_MIN)
    {
      delay(20);
      if(readRawRssiValue() < CHK_RAWRSSI_MIN)
      {  //three primary reads below minimum-check value
        analogRead(RSSI_SEC_PIN);
        delay(20);      //if secondary pin has signal then use it
        if(analogRead(RSSI_SEC_PIN) >= CHK_RAWRSSI_MIN)
          rx5808RssiInPin = RSSI_SEC_PIN;
      }
    }
  }
#endif
}

//Sets min/max-raw-RSSI values for scaling (from analog inputs to 0-100).
void setRx5808RawRssiMinMax(uint16_t minVal, uint16_t maxVal)
{
  rx5808RawRssiMin = minVal;
  rx5808RawRssiMax = maxVal;
}

//Returns min-raw-RSSI value for scaling.
uint16_t getRx5808RawRssiMinVal()
{
  return rx5808RawRssiMin;
}

//Returns max-raw-RSSI value for scaling.
uint16_t getRx5808RawRssiMaxVal()
{
  return rx5808RawRssiMax;
}

//Sets the RX5808 minimum-tune time (in ms).
void setRx5808MinTuneTimeMs(uint8_t val)
{
  rx5808MinTuneTimeMs = val;
}

//Returns the RX5808 minimum-tune time (in ms).
uint8_t getRx5808MinTuneTimeMs()
{
  return rx5808MinTuneTimeMs;
}

//Returns true if the "primary" RSSI-input pin is in use.
boolean isPriRx5808RssiInPinInUse()
{
  return (rx5808RssiInPin == RSSI_PRI_PIN);
}

//Converts two-character frequency code (i.e., "E8") to its corresponding
// frequency value in MHz.
// bandCh:  Band character for frequency code.
// chanCh:  Channel character for frequency code.
// Returns corresponding frequency (MHz) value, or zero if no match.
uint16_t freqCodeCharsToFreqInMhz(char bandCh, char chanCh)
{
  int bandIdx = 0;
  while(true)
  {  //match band-code character to array entry
    if(bandIdx >= NUM_FREQBAND_CODES)
      return 0;         //if no match then return 0
    if(freqBandCodesArray[bandIdx] == bandCh)
      break;
    ++bandIdx;
  }
    //convert numeric digit to offset value (0-7):
  const int numOffs = (int)(chanCh) - ((int)'0' + 1);
  if(numOffs < 0 || numOffs >= CHANNEL_BAND_SIZE)
    return 0;           //if bad value then return 0
    //convert to index in table and return freq value:
  return getChannelFreqTableEntry(bandIdx*CHANNEL_BAND_SIZE + numOffs);
}

//Converts two-character frequency code (i.e., "E8") to its corresponding
// frequency value in MHz.
// codeWordVal:  Two-character code packed into 2-byte word (high byte is
//               band character).
// Returns corresponding frequency (MHz) value, or zero if no match.
uint16_t freqCodeWordToFreqInMhz(uint16_t codeWordVal)
{
  return freqCodeCharsToFreqInMhz((char)(codeWordVal>>(uint16_t)8),
                                                         (char)codeWordVal);
}

//Converts two-character frequency code (i.e., "E8") to a code word.
// codeStr:  String containing band and channel characters for frequency code.
// Returns a two-character code packed into a 2-byte word (high byte is
//  band character).
uint16_t freqCodeStrToCodeWord(const char *codeStr)
{
  const int sLen = strlen(codeStr);
  int p = 0;
  while(codeStr[p] == ' ' && p < sLen)
    ++p;                //ignore any leading spaces
  const char bandCh = (char)toupper(codeStr[p]);
  const char chanCh = (++p < sLen) ? codeStr[p] : ' ';
  return ((uint16_t)bandCh<<(uint16_t)8) + (uint16_t)chanCh;
}

//Converts frequency-index value to its corresponding two-character
// frequency code (i.e., "E8").
// freqIdx:  Frequency-table-index value.
// outStr:  If not null then an output buffer to receive the two-character
//          code (or "??" of no match).
// Returns the two-character code packed into a 2-byte word (high byte is
//   band character), or zero if no match.
uint16_t freqIdxToFreqCode(int freqIdx, char *outStr)
{
  char bandCh,offsCh;
  uint16_t retVal;
  if(freqIdx >= 0 && freqIdx <= CHANNEL_MAX_INDEX)
  {  //index value OK; convert to band and offset characters
    bandCh = freqBandCodesArray[freqIdx/CHANNEL_BAND_SIZE];
    offsCh = (char)(freqIdx%CHANNEL_BAND_SIZE + (int)'1');
         //pack two characters into returned word:
    retVal = (((uint16_t)bandCh) << 8) | offsCh;
  }
  else
  {  //no match or index value out of range
    bandCh = offsCh = '?';
    retVal = 0;
  }
  if(outStr != NULL)
  {  //output-buffer pointer was given; fill in two characters
    outStr[0] = bandCh;
    outStr[1] = offsCh;
  }
  return retVal;
}

//Converts frequency value in MHz to its corresponding two-character
// frequency code (i.e., "E8").
// freqVal:  Frequency value in MHz.
// outStr:  If not null then an output buffer to receive the two-character
//          code (or "??" of no match).
// Returns the two-character code packed into a 2-byte word (high byte is
//   band character), or zero if no match.
uint16_t freqInMhzToFreqCode(uint16_t freqVal, char *outStr)
{
  return freqIdxToFreqCode(getIdxForFreqInMhz(freqVal),outStr);
}

//Converts frequency value in MHz to its corresponding two-character
// frequency code (i.e., "E8") or next/nearest code.
// freqVal:  Frequency value in MHz.
// upFlag:  true to increment when finding next/nearest code; false to
//          decrement.
// outStr:  If not null then an output buffer to receive the two-character
//          code (or "??" of no match).
// Returns the two-character code packed into a 2-byte word (high byte is
//   band character), or zero if no match.
uint16_t freqInMhzToNearestFreqCode(uint16_t freqVal, boolean upFlag,
                                                               char *outStr)
{
  int freqIdx, fChkVal = freqVal;
  uint16_t codeVal;
  do
  {
    freqIdx = getIdxForFreqInMhz(fChkVal);
    if(freqIdx >= 0)
    {  //table-index value found for frequency; check for matching code
      codeVal = freqIdxToFreqCode(freqIdx,outStr);
      if(codeVal > 0)             //if code for freq found then
        return codeVal;           //return code
    }
    if(upFlag)
    {  //increment
      if(++fChkVal > MAX_CHANNEL_MHZ)       //increment frequency by 1 MHz
        fChkVal = MIN_CHANNEL_MHZ;          //if beyond max, wrap to min
    }
    else
    {  //decrement
      if(--fChkVal < MIN_CHANNEL_MHZ)       //decrement frequency by 1 MHz
        fChkVal = MAX_CHANNEL_MHZ;          //if beyond min, wrap to max
    }
  }
  while(fChkVal != freqVal);   //loop unless wrapped around to original freq
  return (uint16_t)0;
}

//Increments (or decrements) band or channel on given two-character
// frequency code (i.e., "E8").
// codeWordVal:  Two-character code packed into 2-byte word (high byte is
//               band character).
// bandFlag:  true for band; false for channel.
// upFlag:  true to increment; false to decrement.
// Returns the two-character code packed into a 2-byte word (high byte is
//   band character).
uint16_t incDecFreqCodeValue(uint16_t codeWordVal, boolean bandFlag,
                                                             boolean upFlag)
{
  char bandCh = (char)(codeWordVal>>(uint16_t)8);
  char chanCh = (char)codeWordVal;
  if(bandFlag)
  {  //increment or decrement band character, with wrap-around
    int bandIdx = 0;
    while(true)
    {  //match band-code character to array entry
      if(bandIdx >= NUM_FREQBAND_CODES)
      {  //no more entries in array to check (no match)
        bandIdx = 0;    //use first one
        break;          //exit loop
      }
      if(freqBandCodesArray[bandIdx] == bandCh)
      {  //array-entry match found
        if(upFlag)
        {  //increment
          if(++bandIdx >= NUM_FREQBAND_CODES)    //increment to next entry
            bandIdx = 0;       //if beyond last then wrap-around to first
        }
        else
        {  //decrement
          if(--bandIdx < 0 )      //decrement to next entry, with wrap-around
            bandIdx = NUM_FREQBAND_CODES - 1;
        }
        break;          //exit loop
      }
      ++bandIdx;
    }
    bandCh = freqBandCodesArray[bandIdx];        //new band character
  }
  else
  {  //increment or decrement channel character, with wrap-around
    if(upFlag)
    {  //increment
      if(++chanCh > (char)(CHANNEL_BAND_SIZE+'0'))
        chanCh = '1';
    }
    else
    {  //decrement
      if(--chanCh < '1')
        chanCh = (char)(CHANNEL_BAND_SIZE+'0');
    }
  }
  return (((uint16_t)bandCh)<<(uint16_t)8) | chanCh;
}
