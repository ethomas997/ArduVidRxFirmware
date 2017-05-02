//FreqListPresets.cpp:  Manager for frequency-list presets.
//
// 12/17/2016 -- [ET]
//

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "Config.h"
#include "Rx5808Fns.h"

    //list of names for presets:
const char freqListNamesPArray[] PROGMEM = "A,B,E,F,R,L,IMD5,IMD6,"
           "ET5,ET5A,ET5B,ET5C,ETBest6,ET6minus1,ETFEAL6,ETFEAL7,ETFEAL8\0";

    //frequency-index values for presets
    // (used 'processListTranslateInfoCmd()' to generate values):
const byte arr00[] PROGMEM = { 0, 1, 2, 3, 4, 5, 6, 7, 255 };         //A
const byte arr01[] PROGMEM = { 8, 9, 10, 11, 12, 13, 14, 15, 255 };   //B
const byte arr02[] PROGMEM = { 16, 17, 18, 19, 20, 21, 22, 23, 255 }; //E
const byte arr03[] PROGMEM = { 24, 25, 26, 27, 28, 29, 30, 31, 255 }; //F
const byte arr04[] PROGMEM = { 32, 33, 34, 35, 36, 37, 38, 39, 255 }; //R
const byte arr05[] PROGMEM = { 40, 41, 42, 43, 44, 45, 46, 47, 255 }; //L
const byte arr06[] PROGMEM = { 17, 25, 27, 30, 21, 255 };             //IMD5
const byte arr07[] PROGMEM = { 19, 17, 25, 27, 30, 21, 255 };         //IMD6
const byte arr08[] PROGMEM = { 18, 7, 28, 30, 23, 255 };              //ET5
const byte arr09[] PROGMEM = { 18, 9, 27, 15, 21, 255 };              //ET5A
const byte arr10[] PROGMEM = { 18, 9, 27, 0, 21, 255 };               //ET5B
const byte arr11[] PROGMEM = { 18, 25, 27, 0, 21, 255 };              //ET5C
const byte arr12[] PROGMEM = { 19, 17, 25, 3, 21, 23, 255 };          //ETBest6
const byte arr13[] PROGMEM = { 19, 17, 25, 21, 23, 255 };             //ET6minus1
const byte arr14[] PROGMEM = { 40, 44, 17, 25, 21, 23, 255 };         //ETFEAL6
const byte arr15[] PROGMEM = { 40, 44, 17, 25, 3, 21, 23, 255 };      //ETFEAL7
const byte arr16[] PROGMEM = { 40, 44, 19, 17, 25, 3, 21, 23, 255 };  //ETFEAL8

    //array of pointers to index-values arrays:
const byte* const freqListValuesPArray[] PROGMEM = { arr00, arr01, arr02,
       arr03, arr04, arr05, arr06, arr07, arr08, arr09, arr10, arr11, arr12,
                                               arr13, arr14, arr15, arr16 };


//Returns array index for given preset name, or -1 if no match.
int indexForPresetName(const char *nameStr)
{
  int idx = 0, p = 0, namePos = 0;
  char ch;
  ch = (char)pgm_read_byte_near(freqListNamesPArray+p);
  while(true)
  {
    if(ch == '\0')
      return -1;        //if reached null then no match
    if(ch == ',')
    {  //found names separator
      ++idx;
      namePos = 0;
      ++p;
      ch = (char)pgm_read_byte_near(freqListNamesPArray+p);
    }
    else if(toupper(ch) == toupper(nameStr[namePos]))
    {  //character matches
      ++p;
      if((ch=(char)pgm_read_byte_near(freqListNamesPArray+p)) == ',' ||
                                                                 ch == '\0')
      {  //at end of preset-name string
        if(!isAlphaNumeric(nameStr[++namePos]))
        {  //also at end of given-name string
          break;        //name match found
        }
      }
      else  //not at end of preset-name string
        ++namePos;      //increment to check next character
    }
    else
    {  //character does not match
      do      //no match; scan through rest of current preset-name string
        ++p;
      while((ch=(char)pgm_read_byte_near(freqListNamesPArray+p)) != ',' &&
                                                                 ch != '\0');
    }
  }
  return idx;
}

//Shows the frequency set for the given preset index (output to serial port).
// Returns number of frequencies in set.
int showFreqSetForPresetIdx(int presetIdx)
{
  const byte* const valuesArrPtr =      //get ptr to array of freq-idx values
            (byte* const)pgm_read_word_near(freqListValuesPArray+presetIdx);
  int i = 0;
  byte btVal;
  while(true)
  {  //for each frequency-index value; fetch value from array
    btVal = pgm_read_byte_near(valuesArrPtr+i);
    if(btVal > (byte)CHANNEL_MAX_INDEX)     //if hit 255 terminator then
      break;                                //exit loop
    Serial.print(' ');
    Serial.print((int)getChannelFreqTableEntry(btVal));    //conv idx to freq
    if(++i >= 255)           //if too many entries then
      break;                 //exit loop (shouldn't happen)
  }
  Serial.println();
  return i;
}

//Shows the frequency set for the given preset name (output to serial port).
// Returns number of frequencies in set, or 0 if name not matched.
int freqListPresetShowForName(const char *nameStr)
{
  const int presetIdx = indexForPresetName(nameStr);  //get index for name
  if(presetIdx < 0)      //if no match then
    return 0;      //return 0 for none
  return showFreqSetForPresetIdx(presetIdx);
}

//Loads the frequency set for the given preset name into the given array.
// nameStr:  Preset name.
// freqArr:  Array to receive frequency values (MHz).
// maxCount:  Maximum number of entries to load (must be > 0).
// Returns number of frequencies loaded, or 0 if name not matched.
int freqListPresetLoadByName(const char *nameStr, uint16_t *freqArr,
                                                               int maxCount)
{
  const int presetIdx = indexForPresetName(nameStr);  //get index for name
  if(presetIdx < 0)      //if no match then
    return 0;      //return 0 for none
  const byte* const valuesArrPtr =      //get ptr to array of freq-idx values
            (byte* const)pgm_read_word_near(freqListValuesPArray+presetIdx);
  int i = 0;
  byte btVal;
  while(true)
  {  //for each frequency-index value; fetch value from array
    btVal = pgm_read_byte_near(valuesArrPtr+i);
    if(btVal > (byte)CHANNEL_MAX_INDEX)     //if hit 255 terminator then
      break;                                //exit loop
    freqArr[i] = getChannelFreqTableEntry(btVal);  //conv idx to freq & store
    if(++i >= maxCount)      //if too many entries then
      break;                 //exit loop
  }
  return i;
}

//Shows all frequency sets (output to serial port).
void freqListPresetShowAllSets()
{
  int idx = 0, p = 0;
  char ch;
  Serial.println(F(" Frequency-list presets:"));
  Serial.print(' ');         //start first line with space
  while(true)
  {
    ch = (char)pgm_read_byte_near(freqListNamesPArray+p);
    if(ch != ',' && ch != '\0')
    {  //not names separator and terminator char
      Serial.print(ch);
    }
    else
    {  //found names separator
      Serial.print(": ");
      showFreqSetForPresetIdx(idx);
      if(ch == '\0')
        break;
      ++idx;
      Serial.print(' ');     //start next line with space
    }
    ++p;
  }
}
