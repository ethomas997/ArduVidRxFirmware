//Display7Seg.cpp:  Support code for pair of 7-segment displays.
//
//  12/3/2016 -- [ET]
//

#include <Arduino.h>
#include "TimerOne.h"
#include "Config.h"
#include "Display7Seg.h"

#if DISP7SEG_ENABLED_FLAG

#define DISP7SEG_ISRINTERVAL_MS 5      //interval for interrupt routine
#define DISP7SEG_BITMSK_UNDEF ((byte)0b00001000)      //mask for "undefined"
#define DISP7SEG_BITMSK_DP ((byte)0b10000000)         //mask for decimal pt
#define DISP7SEG_BITMSKARR_MINVAL 32        //min displayable ASCII value
#define DISP7SEG_BITMSKARR_MAXVAL 127       //max displayable ASCII value
#define DISP7SEG_BITMSKARR_LEN (DISP7SEG_BITMSKARR_MAXVAL-DISP7SEG_BITMSKARR_MINVAL+1)
#define DISP7SEG_DISPWORDSARR_SIZE 30       //size for 'disp7SegDisplayWordsArr'
#define DISP7SEG_DISPWORDSINTVL_MS 100      //interval btw displayed bitmasks

    //array to convert ASCII codes to 7-segment-bitmask values:
byte disp7SegAsciiToBitmaskArr[DISP7SEG_BITMSKARR_LEN];

volatile byte disp7SegLeftMaskOut = (byte)0;     //left display bitmask
volatile byte disp7SegRightMaskOut = (byte)0;    //right display bitmask
volatile boolean disp7SegLeftActiveFlag = false;      //toggle L/R
boolean disp7SegDataUpdatingFlag = false;        //flag for thread safe

    //value, duration and end-time for initial-display values:
volatile uint32_t disp7SegInitDispBitmaskUint32 = 0;
volatile boolean disp7SegInitDispBitmaskFlag = false;
volatile int disp7SegInitDispDurationMs = 0;
volatile unsigned long disp7SegInitDispEndTime = 0;

    //value and end-time for override-display values:
volatile uint16_t disp7SegOvrDispBitmaskWord = 0;
volatile int disp7SegOvrDispDurationMs = 0;
volatile unsigned long disp7SegOvrDispEndTime = 0;

    //array of left/right-bitmask words to be sent to displays:
uint16_t disp7SegDisplayWordsArr[DISP7SEG_DISPWORDSARR_SIZE];
int disp7SegDisplayWordsNumEntries = 0;
volatile int disp7SegDisplayWordsCurIdx = 0;
volatile unsigned long disp7SegDisplayWordsNextTime = 0;


//Sets a 'disp7Seg_asciiToBitmsk[]' array entry .
// ch:  ASCII value for entry.
// mskVal:  7-segment-bitmask value for entry.
void setAsciiToBitmskEntry(char ch, byte mskVal)
{
  if(ch >= (char)DISP7SEG_BITMSKARR_MINVAL && ch <= DISP7SEG_BITMSKARR_MAXVAL)
  {  //ASCII value is in range
    disp7SegAsciiToBitmaskArr[ch-(char)DISP7SEG_BITMSKARR_MINVAL] = mskVal;
  }
}

//Returns bitmask for given ASCII code.
byte disp7SegAsciiToBitmask(char ch)
{
  return (ch >= (char)DISP7SEG_BITMSKARR_MINVAL &&
                                          ch <= DISP7SEG_BITMSKARR_MAXVAL) ?
             disp7SegAsciiToBitmaskArr[ch-(char)DISP7SEG_BITMSKARR_MINVAL] :
                                                      DISP7SEG_BITMSK_UNDEF;
}

//Generates and returns a two-byte word for the given values.
// leftCh:  ASCII-character value for left display.
// leftDpFlag:  true to show decimal point on left display.
// rightCh:  ASCII-character value for right display.
// rightDpFlag:  true to show decimal point on right display.
// Returns:  A two-byte word with the left-display bitmask in the upper
//           byte and the right-display bitmask in the lower byte.
uint16_t disp7SegConvAsciiCharsToWord(char leftCh, boolean leftDpFlag,
                                          char rightCh, boolean rightDpFlag)
{
  const byte leftMask = disp7SegAsciiToBitmask(leftCh) |
                                (leftDpFlag ? DISP7SEG_BITMSK_DP : (byte)0);
  const byte rightMask = disp7SegAsciiToBitmask(rightCh) |
                               (rightDpFlag ? DISP7SEG_BITMSK_DP : (byte)0);
  return (((uint16_t)leftMask)<<(uint16_t)8) | rightMask;
}

//Sets the displays to the given initial values; overrides values displayed
// via 'disp7SegEnterToDisplayWordsArr()' or 'disp7SegSetOvrAsciiValues()'.
// The leftCh1/rightCh1 values are shown first, the leftCh2/rightCh2 values
// are shown second.
// leftCh:  ASCII-character value for left display.
// leftDpFlag:  true to show decimal point on left display.
// rightCh:  ASCII-character value for right display.
// rightDpFlag:  true to show decimal point on right display.
// dispTimeMs:  display time in milliseconds (non-zero).
void disp7SegSetInitAsciiValues(char leftCh1, boolean leftDpFlag1,
                                        char rightCh1, boolean rightDpFlag1,
                                          char leftCh2, boolean leftDpFlag2,
                                        char rightCh2, boolean rightDpFlag2,
                                                             int dispTimeMs)
{
  disp7SegDataUpdatingFlag = true;     //indicate data update in progress
  disp7SegInitDispDurationMs = dispTimeMs;        //save duration time
  disp7SegInitDispEndTime = 0;                    //clear time for now
         //setup initial bitmask values (2 words):
  disp7SegInitDispBitmaskUint32 = (((uint32_t)disp7SegConvAsciiCharsToWord(
                        leftCh2,leftDpFlag2,rightCh2,rightDpFlag2)) << 16) |
                           disp7SegConvAsciiCharsToWord(leftCh1,leftDpFlag1,
                                                     rightCh1,rightDpFlag1);
                                       //indicate bitmask setup:
  disp7SegInitDispBitmaskFlag = (disp7SegInitDispBitmaskUint32 != 0);
  disp7SegDisplayWordsCurIdx = 0;      //make sure start at initial index
  disp7SegDisplayWordsNextTime = 0;    //initialize next-entry time
  disp7SegDataUpdatingFlag = false;    //indicate data update completed
}

//Sets the displays to the given values; overrides values displayed via
// 'disp7SegEnterToDisplayWordsArr()'.
// leftCh:  ASCII-character value for left display.
// leftDpFlag:  true to show decimal point on left display.
// rightCh:  ASCII-character value for right display.
// rightDpFlag:  true to show decimal point on right display.
// dispTimeMs:  display time in ms, or 0 for indefinite.
void disp7SegSetOvrAsciiValues(char leftCh, boolean leftDpFlag, char rightCh,
                                        boolean rightDpFlag, int dispTimeMs)
{
  disp7SegDataUpdatingFlag = true;     //indicate data update in progress
  disp7SegOvrDispDurationMs = dispTimeMs;        //save duration time
  disp7SegOvrDispEndTime = 0;                    //clear time for now
         //setup override bitmask values:
  disp7SegOvrDispBitmaskWord = disp7SegConvAsciiCharsToWord(
                                     leftCh,leftDpFlag,rightCh,rightDpFlag);
  disp7SegDisplayWordsCurIdx = 0;      //reset so will resume at start
  disp7SegDisplayWordsNextTime = 0;    //initialize next-entry time
  disp7SegDataUpdatingFlag = false;    //indicate data update completed
}

//Sets the displays to the given values; overrides values displayed via
// 'disp7SegEnterToDisplayWordsArr()'.
// wordVal:  Two ASCII-characters values encoded into a 2-byte word .
// dispTimeMs:  display time in ms, or 0 for indefinite.
void disp7SegSetOvrAsciiViaWord(uint16_t wordVal, int dispTimeMs)
{
  disp7SegSetOvrAsciiValues((char)(wordVal>>(uint16_t)8),false,
                           (char)(wordVal&(uint16_t)0x7F),false,dispTimeMs);
}

//Sets the displays to two dashes ("--"); overrides values displayed via
// 'disp7SegEnterToDisplayWordsArr()'.
// dispTimeMs:  display time in ms, or 0 for indefinite.
void disp7SegSetOvrShowDashes(int dispTimeMs)
{
  disp7SegSetOvrAsciiValues('-',false,'-',false,dispTimeMs);
}

//Clears any previous override display.
void disp7SegClearOvrDisplay()
{
  disp7SegSetOvrAsciiValues(' ',false,' ',false,0);
}

//Enters instances of the given bitmask-words into the
// 'disp7SegDisplayWordsArr[]' array.  Each 'count' value
// specifies the number of instances for the 'word' value.
void disp7SegEnterToDisplayWordsArr(uint16_t word1, int count1,
                     uint16_t word2, int count2, uint16_t word3, int count3)
{
  int c, firstIdx = 0, arrIdx = 0;
  disp7SegDataUpdatingFlag = true;     //indicate data update in progress
  c = 0;
  while(++c <= count1)
  {  //for each instance of the bitmask-word to be entered
    if(arrIdx >= DISP7SEG_DISPWORDSARR_SIZE)
      break;      //if beyond array size then exit loop
              //enter an instance of the bitmask-word:
    disp7SegDisplayWordsArr[arrIdx++] = word1;
  }
         //if first bitmask-word blank and is initial display then
         // setup to show second one first (don't start with blank):
  if(word1 == (uint16_t)0 && disp7SegDisplayWordsCurIdx == 0)
    firstIdx = arrIdx;
  c = 0;
  while(++c <= count2)
  {  //for each instance of the bitmask-word to be entered
    if(arrIdx >= DISP7SEG_DISPWORDSARR_SIZE)
      break;      //if beyond array size then exit loop
              //enter an instance of the bitmask-word:
    disp7SegDisplayWordsArr[arrIdx++] = word2;
  }
         //if first bitmask-word blank and not initial display then
         // setup to show last one first (track fast frequency changes):
  if(word1 == (uint16_t)0 && disp7SegDisplayWordsCurIdx > 0)
    firstIdx = arrIdx;            //setup to show last one first
  c = 0;
  while(++c <= count3)
  {  //for each instance of the bitmask-word to be entered
    if(arrIdx >= DISP7SEG_DISPWORDSARR_SIZE)
      break;      //if beyond array size then exit loop
              //enter an instance of the bitmask-word:
    disp7SegDisplayWordsArr[arrIdx++] = word3;
  }
  disp7SegDisplayWordsNumEntries = arrIdx;  //set # of bitmask-words entered
         //reset start index (skip first bitmask-word if blank and next OK):
  disp7SegDisplayWordsCurIdx = (firstIdx < arrIdx) ? firstIdx : 0;
  disp7SegDisplayWordsNextTime = 0;         //initialize next-entry time
  disp7SegDataUpdatingFlag = false;         //indicate data update completed
}

//ISR Timer Routine for updating displays.
void disp7SegTimerIsr()
{
         //toggle left/right display active:
  disp7SegLeftActiveFlag = !disp7SegLeftActiveFlag;
  if(!disp7SegDataUpdatingFlag)
  {  //data update is not currently in progress
    const unsigned long curTimeMs = millis();
    if(disp7SegInitDispBitmaskFlag)
    {  //initial display values should be shown
      if(disp7SegInitDispDurationMs > 0)
      {  //duration not zero
        if(curTimeMs >= disp7SegInitDispEndTime)
        {  //duration end-time reached (or first time through)
          if(disp7SegInitDispEndTime > 0)
          {  //not first time through; shift to next word
            disp7SegInitDispBitmaskUint32 >>= 16;
          }
          if(disp7SegInitDispBitmaskUint32 != 0)
          {  //initial-display values (still) available
            const uint16_t dispWord = (uint16_t)disp7SegInitDispBitmaskUint32;
                  //set output bitmasks to initial values:
            disp7SegLeftMaskOut = (byte)(dispWord >> (uint16_t)8);
            disp7SegRightMaskOut = (byte)dispWord;
                  //set end time for display of values:
            disp7SegInitDispEndTime = millis() + disp7SegInitDispDurationMs;
          }
          else  //no more initial-display values
            disp7SegInitDispBitmaskFlag = false;
        }
      }
      else  //duration is zero; just clear flag
        disp7SegInitDispBitmaskFlag = false;
    }
    else if(disp7SegOvrDispBitmaskWord != (uint16_t)0)
    {  //display override value setup
      if(disp7SegOvrDispDurationMs > 0)
      {  //duration not zero for indefinite
        if(disp7SegOvrDispEndTime == 0)
        {  //display-override end time not yet setup; set it now
          disp7SegOvrDispEndTime = millis() + disp7SegOvrDispDurationMs;
                   //set output bitmask values to override values:
          disp7SegLeftMaskOut =
                          (byte)(disp7SegOvrDispBitmaskWord >> (uint16_t)8);
          disp7SegRightMaskOut = (byte)disp7SegOvrDispBitmaskWord;
        }
        else
        {  //non-indefinite display-override is in progress
          if(curTimeMs >= disp7SegOvrDispEndTime)     //if end time reached
            disp7SegOvrDispBitmaskWord = (uint16_t)0; // then clear override
        }
      }
      else
      {  //indefinite duration; set output bitmask values to override values
        disp7SegLeftMaskOut =
                          (byte)(disp7SegOvrDispBitmaskWord >> (uint16_t)8);
        disp7SegRightMaskOut = (byte)disp7SegOvrDispBitmaskWord;
      }
    }
    else if(disp7SegDisplayWordsNumEntries > 0)
    {  //display init/override values not setup and display array not empty
      if(curTimeMs >= disp7SegDisplayWordsNextTime)
      {  //time reached for next entry in display array (or first)
        const uint16_t dispWord =
                        disp7SegDisplayWordsArr[disp7SegDisplayWordsCurIdx];
              //set output bitmasks to values from display array:
        disp7SegLeftMaskOut = (byte)(dispWord >> (uint16_t)8);
        disp7SegRightMaskOut = (byte)dispWord;
              //increment display-array index, with wrap-around:
        if(++disp7SegDisplayWordsCurIdx >= disp7SegDisplayWordsNumEntries)
          disp7SegDisplayWordsCurIdx = 0;
              //setup next-entry time:
        disp7SegDisplayWordsNextTime = curTimeMs + DISP7SEG_DISPWORDSINTVL_MS;
      }
    }

    byte dispMsk;
    if(disp7SegLeftActiveFlag)
    {  //left display will now be active
      digitalWrite(DISP7SEG_SELRIGHT_PIN,HIGH);  //turn off right display
      dispMsk = disp7SegLeftMaskOut;
    }
    else
    {  //right display will now be active
      digitalWrite(DISP7SEG_SELLEFT_PIN,HIGH);   //turn off left display
      dispMsk = disp7SegRightMaskOut;
    }
           //write bitmask value to display-segment outputs:
    digitalWrite(DISP7SEG_A_PIN,
                ((((dispMsk) & (byte)0b00000001) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_B_PIN,
                ((((dispMsk) & (byte)0b00000010) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_C_PIN,
                ((((dispMsk) & (byte)0b00000100) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_D_PIN,
                ((((dispMsk) & (byte)0b00001000) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_E_PIN,
                ((((dispMsk) & (byte)0b00010000) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_F_PIN,
                ((((dispMsk) & (byte)0b00100000) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_G_PIN,
                ((((dispMsk) & (byte)0b01000000) != (byte)0) ? LOW : HIGH));
    digitalWrite(DISP7SEG_DP_PIN,
                ((((dispMsk) & (byte)0b10000000) != (byte)0) ? LOW : HIGH));
    if(disp7SegLeftActiveFlag)
      digitalWrite(DISP7SEG_SELLEFT_PIN,LOW);    //turn on left display
    else                                              // or
      digitalWrite(DISP7SEG_SELRIGHT_PIN,LOW);   //turn on right display
  }
  else
  {  //data update is in progress; just do select-lines toggle
    if(disp7SegLeftActiveFlag)
    {  //left display will now be active
      digitalWrite(DISP7SEG_SELRIGHT_PIN,HIGH);  //turn off right display
      digitalWrite(DISP7SEG_SELLEFT_PIN,LOW);    //turn on left display
    }
    else
    {  //right display will now be active
      digitalWrite(DISP7SEG_SELLEFT_PIN,HIGH);   //turn off left display
      digitalWrite(DISP7SEG_SELRIGHT_PIN,LOW);   //turn on right display
    }
  }
}

//Sets up resources for display management, does hardware setup for Arduino
// output pins, and starts timer interrupts.
void disp7SegSetup()
{
    //setup array to convert ASCII codes to 7-segment-bitmask values
              //initialize all values to "undefined" value:
  for(int i=0; i<DISP7SEG_BITMSKARR_LEN; ++i)
    disp7SegAsciiToBitmaskArr[i] = DISP7SEG_BITMSK_UNDEF;

  setAsciiToBitmskEntry('0',0b00111111);
  setAsciiToBitmskEntry('1',0b00000110);
  setAsciiToBitmskEntry('2',0b01011011);
  setAsciiToBitmskEntry('3',0b01001111);
  setAsciiToBitmskEntry('4',0b01100110);
  setAsciiToBitmskEntry('5',0b01101101);
  setAsciiToBitmskEntry('6',0b01111101);
  setAsciiToBitmskEntry('7',0b00000111);
  setAsciiToBitmskEntry('8',0b01111111);
  setAsciiToBitmskEntry('9',0b01101111);
  setAsciiToBitmskEntry('A',0b01110111);
  setAsciiToBitmskEntry('a',0b01011111);
  setAsciiToBitmskEntry('b',0b01111100);
  setAsciiToBitmskEntry('B',0b01111100);
  setAsciiToBitmskEntry('c',0b01011000);
  setAsciiToBitmskEntry('C',0b00111001);
  setAsciiToBitmskEntry('D',0b01011110);
  setAsciiToBitmskEntry('d',0b01011110);
  setAsciiToBitmskEntry('E',0b01111001);
  setAsciiToBitmskEntry('e',0b01111001);
  setAsciiToBitmskEntry('F',0b01110001);
  setAsciiToBitmskEntry('f',0b01110001);
  setAsciiToBitmskEntry('H',0b01110110);
  setAsciiToBitmskEntry('h',0b01110100);
  setAsciiToBitmskEntry('I',0b00000110);
  setAsciiToBitmskEntry('i',0b00000100);
  setAsciiToBitmskEntry('J',0b00001110);
  setAsciiToBitmskEntry('j',0b00001100);
  setAsciiToBitmskEntry('L',0b00111000);
  setAsciiToBitmskEntry('l',0b00111000);
  setAsciiToBitmskEntry('n',0b01010100);
  setAsciiToBitmskEntry('N',0b01010100);
  setAsciiToBitmskEntry('O',0b00111111);
  setAsciiToBitmskEntry('o',0b01011100);
  setAsciiToBitmskEntry('P',0b01110011);
  setAsciiToBitmskEntry('p',0b01110011);
  setAsciiToBitmskEntry('r',0b01010000);
  setAsciiToBitmskEntry('R',0b01010000);
  setAsciiToBitmskEntry('t',0b01111000);
  setAsciiToBitmskEntry('T',0b01111000);
  setAsciiToBitmskEntry('U',0b00111110);
  setAsciiToBitmskEntry('u',0b00011100);
  setAsciiToBitmskEntry('y',0b01101110);
  setAsciiToBitmskEntry('Y',0b01101110);
  setAsciiToBitmskEntry(' ',0b00000000);
  setAsciiToBitmskEntry('.',DISP7SEG_BITMSK_DP);
  setAsciiToBitmskEntry('-',0b01000000);
  setAsciiToBitmskEntry('=',0b01001000);
  setAsciiToBitmskEntry('[',0b00111001);
  setAsciiToBitmskEntry(']',0b00001111);
  setAsciiToBitmskEntry('\'',0b00000010);
  setAsciiToBitmskEntry('`',0b00100000);
  setAsciiToBitmskEntry('\"',0b00100010);
  setAsciiToBitmskEntry('/',0b01010010);
  setAsciiToBitmskEntry('\\',0b01100100);
  setAsciiToBitmskEntry('|',0b00110000);
  setAsciiToBitmskEntry('_',0b00001000);

    //enable output pins for display segments:
  pinMode(DISP7SEG_A_PIN,OUTPUT);
  pinMode(DISP7SEG_B_PIN,OUTPUT);
  pinMode(DISP7SEG_C_PIN,OUTPUT);
  pinMode(DISP7SEG_D_PIN,OUTPUT);
  pinMode(DISP7SEG_E_PIN,OUTPUT);
  pinMode(DISP7SEG_F_PIN,OUTPUT);
  pinMode(DISP7SEG_G_PIN,OUTPUT);
  pinMode(DISP7SEG_DP_PIN,OUTPUT);
    //enable output pins for selecting left or right digit:
  pinMode(DISP7SEG_SELLEFT_PIN,OUTPUT);
  pinMode(DISP7SEG_SELRIGHT_PIN,OUTPUT);

  digitalWrite(DISP7SEG_SELLEFT_PIN,HIGH);       //initialize both off
  digitalWrite(DISP7SEG_SELRIGHT_PIN,HIGH);

  Timer1.initialize(DISP7SEG_ISRINTERVAL_MS*1000L);   //set timer interval
  Timer1.attachInterrupt(disp7SegTimerIsr );       //attach service fn

         //set some initial values for displays:
  disp7SegLeftMaskOut = disp7SegRightMaskOut = disp7SegAsciiToBitmask('0');
}

//Disconnects timer interrupt.
void disp7SegShutdown()
{
  Timer1.detachInterrupt();
}

//Determines mode for given pin.  Code from:
// http://arduino.stackexchange.com/questions/13165/how-to-read-pinmode-for-digital-pin
// Returns:  OUTPUT, INPUT, INPUT_PULLUP, or -1 (if error).
int determinePinMode(uint8_t pin)
{
  if (pin >= NUM_DIGITAL_PINS) return (-1);

  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg = portModeRegister(port);
  if (*reg & bit) return (OUTPUT);

  volatile uint8_t *out = portOutputRegister(port);
  return ((*out & bit) ? INPUT_PULLUP : INPUT);
}

//Determines if the given pin is connected to a display-segment circuit
// (with a pullup).
// Returns true if connected; false if not.
boolean disp7SegTestPinConnected(uint8_t pinId)
{
  pinMode(pinId,OUTPUT);               //set pin to output
  digitalWrite(pinId,LOW);             //drive pin low
  pinMode(pinId,INPUT);                //set pin to input
  delay(5);                            //brief delay for pullup action
  return (digitalRead(pinId) != LOW);  //check if pin now high
}

//Determines if a display is connected.
// Returns true if connected; false if not.
boolean disp7SegTestDisplayConnected()
{
  const int oldLPinMode = determinePinMode(DISP7SEG_SELLEFT_PIN);
  const int oldRPinMode = determinePinMode(DISP7SEG_SELRIGHT_PIN);
    //enable selectors to get pullup effect on segments:
  pinMode(DISP7SEG_SELLEFT_PIN,OUTPUT);
  pinMode(DISP7SEG_SELRIGHT_PIN,OUTPUT);
  digitalWrite(DISP7SEG_SELLEFT_PIN,LOW);
  digitalWrite(DISP7SEG_SELRIGHT_PIN,LOW);
    //check three pins; indicate connected if any look like they are:
  const boolean retFlag = disp7SegTestPinConnected(DISP7SEG_B_PIN) |
                                  disp7SegTestPinConnected(DISP7SEG_D_PIN) |
                                   disp7SegTestPinConnected(DISP7SEG_F_PIN);
  pinMode(DISP7SEG_SELLEFT_PIN,oldLPinMode);     //restore pin modes
  pinMode(DISP7SEG_SELRIGHT_PIN,oldRPinMode);
  return retFlag;
}

#endif  //DISP7SEG_ENABLED_FLAG
