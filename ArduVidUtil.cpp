//ArduVidUtil.cpp:  Utility functions.
//
//  2/18/2017 -- [ET]
//

#include <Arduino.h>
#include <EEPROM.h>
#include "ArduVidUtil.h"

boolean serialEchoFlag = true;         //global flag for serial-echo mode

char serialInputBuffer[RECV_BUFSIZ] = { '\0' };
int serialInputBuffPos = 0;
boolean serialInputPromptFlag = true;
uint16_t serialInputLastTwoChars = 0;
char lastCommandChar = '\0';
boolean serialDoReportRssiFlag = false;

volatile byte trackedD2InputState = HIGH, trackedD3InputState = HIGH;
volatile byte triggerD2LiveCounter = 0, triggerD3LiveCounter = 0;


//Reads and returns the next serial character if it is available quickly;
// returns '\0' if none available.
char readNextEscKeyChar()
{
  for(int i=0; i<10; ++i)
  {
    if(Serial.available())
      return (char)Serial.read();
    delay(10);
  }
  return '\0';
}

//Translates escape-key codes to command characters.
char translateEscKeyChar(char inCh)
{
  char outCh;
  switch(inCh)
  {
    case KEY_UP:
      outCh = CMD_KEY_UP;
      break;
    case KEY_DOWN:
      outCh = CMD_KEY_DOWN;
      break;
    case KEY_LEFT:
      outCh = CMD_KEY_LEFT;
      break;
    case KEY_RIGHT:
      outCh = CMD_KEY_RIGHT;
      break;
    case KEY_HOME:
      outCh = CMD_KEY_HOME;
      readNextEscKeyChar();       //clear trailing escape-sequence char
      break;
    case KEY_END:
      outCh = CMD_KEY_END;
      readNextEscKeyChar();       //clear trailing escape-sequence char
      break;
    default:
      outCh = '\0';
  }
  return outCh;
}

//Returns next line of data from the serial port, or NULL if not
// available.
// flushingFlag:  true if input lines of data being flushed (no echo).
char *doRecvNextSerialLine(boolean flushingFlag)
{
         //local serial-echo flag follows global unless flushing:
  const boolean echoFlag = serialEchoFlag && !flushingFlag;
  while(Serial.available())
  {  //loop while serial data is available
    char ch = (char)Serial.read();
    if(serialInputLastTwoChars == KEYSEQ_ESC)
    {  //escape input sequence was received
      serialInputLastTwoChars = 0;          //clear input tracker
      if(serialInputBuffPos == 0)
      {  //no previous characters entered on line input
        if((ch=translateEscKeyChar(ch)) != '\0')
        {  //escape char input recognized and translated OK; enter it
          serialInputBuffer[serialInputBuffPos++] = ch;
          serialInputBuffer[serialInputBuffPos] = '\0';
          if(echoFlag)
            Serial.write((int)ch);
          ch = KEY_CR;       //follow up with <Enter> input
        }
      }
      else
        ch = '\0';
    }
//    Serial.print('{');
//    Serial.print((int)ch);
//    Serial.print('}');
         //check if input is line that begins with '>' or ' '
         // (that should be ignored by slave receiver):
    const boolean ignoreFlag = (serialInputBuffPos > 0 &&
                              (serialInputBuffer[0] == SERIAL_PROMPT_CHAR ||
                              serialInputBuffer[0] == SERIAL_LIGNORE_CHAR));
    if(ch == KEY_CR || ch == KEY_LF)
    {  //end of line
      if(!ignoreFlag)
      {  //input is not line that begins with '>' or ' '
              //if command entered then save command character
              // for use as repeat command via <Enter> key:
        if(serialInputBuffPos > 0)
          lastCommandChar = (char)toupper(serialInputBuffer[0]);
        serialInputBuffer[serialInputBuffPos] = '\0';
        serialInputBuffPos = 0;
        if(serialInputLastTwoChars + ch == (uint16_t)(KEY_CR+KEY_LF))
        {  //input is second char of CRLF sequence; ignore it
          serialInputLastTwoChars = (uint16_t)ch;  //set input tracker to char
          continue;
        }
        if(!flushingFlag)         //if not flushing lines of input data then
          Serial.println();       //send newline to serial
        serialInputPromptFlag = true;
        serialInputLastTwoChars = (uint16_t)ch;  //set input tracker to char
        return serialInputBuffer;
      }
         //input is line that begins with '>' or ' ' (should be ignored)
      serialInputBuffPos = 0;     //clear buffer
      lastCommandChar = '\0';
      serialInputLastTwoChars = (uint16_t)ch;  //set input tracker to char
      if(echoFlag)
      {  //serial echo enabled; erase displayed char ('>' or ' ')
        Serial.write((int)KEY_BACKSP);
        Serial.write((int)' ');
        Serial.write((int)KEY_BACKSP);
      }
      return NULL;
    }
    if(ch >= ' ' && ch <= 'z' && ch != KEY_ESCNXT &&
                                         serialInputBuffPos < RECV_BUFSIZ-2)
    {  //character is valid and enough room in buffer
      if(!ignoreFlag)
      {  //input is not line that begins with '>' or ' '
        serialInputBuffer[serialInputBuffPos++] = ch;    //add received char
        serialInputBuffer[serialInputBuffPos] = '\0';
        if(echoFlag)
          Serial.write((int)ch);
      }
      serialInputLastTwoChars = (uint16_t)ch;    //set input tracker to char
    }
    else if((ch == KEY_BACKSP || ch == KEY_DEL) && serialInputBuffPos > 0)
    {  //handle backspace (or delete code)
      serialInputBuffer[--serialInputBuffPos] = '\0';
      if(echoFlag)
      {
        Serial.write((int)KEY_BACKSP);
        Serial.write((int)' ');
        Serial.write((int)KEY_BACKSP);
      }
      serialInputLastTwoChars = 0;          //clear input tracker
    }
    else if(ch == KEY_REPORT)
    {  //special report-RSSI character
      serialDoReportRssiFlag = true;        //set indicator flag
    }
    else
    {         //track last two (escape) input chars in 16-bit variable:
      serialInputLastTwoChars = (serialInputLastTwoChars<<(uint16_t)8) |
                                                                (uint8_t)ch;
    }
  }
  return NULL;
}

//Returns next line of data from the serial port, or NULL if not
// available.  If there are queued lines of input data not yet received
// then they are discarded (because they were received while the system
// was too busy to process them, and processing a bunch of queued-up
// commands is problematic).
char *getNextSerialLine()
{
  if(serialInputPromptFlag)
  {  //show prompt at start of input line
    serialInputPromptFlag = false;
    Serial.write((int)SERIAL_PROMPT_CHAR);
  }
  return doRecvNextSerialLine(false);
}

//Clears any queued lines of serial-input data.
void flushSerialInputLines()
{
  while(doRecvNextSerialLine(true) != NULL);
  if(serialEchoFlag && serialInputBuffPos > 0)
  {  //echo enabled and buffer contains received data (not followed by LF)
    int i = 0;   //echo received characters now
    do
      Serial.write((int)serialInputBuffer[i]);
    while(++i < serialInputBuffPos);
  }
}

//Returns true if serial-input characters are available in the input buffer.
boolean getSerialInputAvailflag()
{
  return (serialInputBuffPos > 0);
}

//Sets the 'serialInputPromptFlag' so the prompt will be shown on the
// next call to 'getNextSerialLine()'.
void setSerialInputPromptFlag()
{
  serialInputPromptFlag = true;
}

//Clears the 'serialInputPromptFlag' so the prompt will not be shown on the
// next call to 'getNextSerialLine()'.
void clearSerialInputPromptFlag()
{
  serialInputPromptFlag = false;
}

//Returns true if the report-RSSI character has been received via the
// serial port since the last call to this function.
boolean getDoReportRssiFlag()
{
  if(serialDoReportRssiFlag)
  {  //indicator flag was set; clear it and return true
    serialDoReportRssiFlag = false;
    return true;
  }
  return false;
}

//Returns first character of last command line (for possible use as
// repeat command via <Enter> key).
char getLastCommandChar()
{
  return lastCommandChar;
}

//Clears the value returned by 'getLastCommandChar()'.
void clearLastCommandChar()
{
  lastCommandChar = '\0';
}

//Converts given string to integer, validating input.
// Returns true if successful; false if bad input.
boolean convStrToInt(const char *str, int *pIntVal)
{
  int sPos = 0;
  while(str[sPos] == ' ' || str[sPos] == '\t')
    ++sPos;        //skip any leading whitespace
  int ePos = sPos;
  while(str[ePos] >= '0' && str[ePos] <= '9')
    ++ePos;        //scan through digits
  if(ePos <= sPos || ePos-sPos > 5)
    return false;       //validate length is 1 to 5 digits
  *pIntVal = atoi(&str[sPos]);
  return true;
}

//Removes the given value from the given array of values.
// uintArr:  Array of 'uint16_t' values.
// arrCount:  Number of entries in array.
// rmVal:  Value to be removed from array.
// Returns:  New number of entries in array (arrCount-1), or same
//           value if match not found.
int removeValueFromArray(uint16_t *uintArr, int arrCount, uint16_t rmVal)
{
  int p = 0;
  while(p < arrCount)
  {  //for each entry while searching for match
    if(uintArr[p] == rmVal)
    {  //match found
      while(++p < arrCount)       //shift all following entries down by 1
        uintArr[p-1] = uintArr[p];
      return arrCount - 1;        //return given array count - 1
    }
    ++p;
  }
  return arrCount;           //no match; return given array count
}

//Shows the list of integer values in the given array.
void showUint16ArrayList(const uint16_t intArr[], int intArrCount)
{
  if(intArrCount > 0)
  {
    int i = 0;
    while(true)
    {  //for each value in list
      Serial.print((int)intArr[i]);
      if(++i >= intArrCount)
        break;
      Serial.print(",");
    }
  }
}

//Restarts program from beginning (but does not reset peripherals
// and registers).
void doSoftwareReset()
{
  asm volatile ("  jmp 0");
}

//Writes byte to EEPROM at address.
void writeByteToEeprom(int addr, uint8_t val)
{
  EEPROM.write(addr,val);
}

//Reads byte at address from EEPROM.
uint8_t readByteFromEeprom(int addr)
{
  return EEPROM.read(addr);
}

//Writes 2-byte word to EEPROM at address.
void writeWordToEeprom(int addr, uint16_t val)
{
  EEPROM.write(addr,lowByte(val));
  EEPROM.write(addr+1,highByte(val));
}

//Reads 2-byte word at address from EEPROM.
uint16_t readWordFromEeprom(int addr)
{
  const uint8_t lb = EEPROM.read(addr);
  const uint8_t hb = EEPROM.read(addr+1);
  return (((uint16_t)hb) << 8) + lb;
}

//Writes string to EEPROM at address.
// addr:  EEPROM address.
// str:  Null-terminated string of characters.
// fieldLen:  Length (in bytes) of storage field in EEPROM.
void writeStringToEeprom(int addr, const char *str, int fieldLen)
{
  uint8_t btVal;
  int i = 0;
  while(true)
  {  //for each byte written
    btVal = (uint8_t)str[i];
    EEPROM.write(addr+i,btVal);
    if(++i >= fieldLen)      //if entire field filled then
      return;                //exit function
    if(btVal == (uint8_t)0)       //if end of string then
      break;                      //exit loop
  }
  do     //fill rest of field with nulls
    EEPROM.write(addr+i,(uint8_t)0);
  while(++i < fieldLen);
}

//Reads string at address from EEPROM.
// addr:  EEPROM address.
// outStr:  Buffer for string (with a size of at least fieldLen+1).
// fieldLen:  Length (in bytes) of storage field in EEPROM.
// Returns number of characters read.
int readStringFromEeprom(int addr, char *outStr, int fieldLen)
{
  uint8_t btVal;
  int i = 0;
  do
  {  //for each byte read until null or end of field
    btVal = EEPROM.read(addr+i);
    if(btVal == (uint8_t)255)     //if 0xFF (uninitialized) then
      btVal = (uint8_t)0;         //convert to zero
    outStr[i] = btVal;
  }
  while(btVal != (uint8_t)0 && ++i < fieldLen);
  return i;
}

//Determines if string at address from EEPROM is empty.
// Returns true if string at address from EEPROM is empty (or field is
// uninitialized).
boolean isStringFromEepromEmpty(int addr)
{
  const byte btVal = EEPROM.read(addr);
  return (btVal == (uint8_t)0 || btVal == (uint8_t)255);
}

//Displays string at address from EEPROM, sending characters to serial port.
// addr:  EEPROM address.
// fieldLen:  Length (in bytes) of storage field in EEPROM.
// Returns number of characters read.
int showStringFromEeprom(int addr, int fieldLen)
{
  uint8_t btVal;
  int i = 0;
  do
  {  //for each byte read until null or end of field
    btVal = EEPROM.read(addr+i);
    if(btVal == (uint8_t)0 || btVal == (uint8_t)255)
      break;
    Serial.print((char)btVal);
  }
  while(++i < fieldLen);
  return i;
}

//Writes array of uint16 values to EEPROM at address.
// addr:  EEPROM address.
// fieldLen:  Length (in bytes) of storage field in EEPROM.
// uintArr:  Array of uint16 values.
// arrCount:  Number of entries in array.
// Returns true if successful; false if 'arrCount' value out of bounds.
boolean writeUint16ArrayToEeprom(int addr, int fieldLen,
                                            uint16_t *uintArr, int arrCount)
{
  if(arrCount > (fieldLen-2)/2 || arrCount < 0)
    return false;
  writeWordToEeprom(addr,(uint16_t)arrCount);    //write entries count
  int offs = 0;
  for(int i=0; i<arrCount; ++i)
  {  //for each entry; write to EEPROM location
    offs += 2;
    writeWordToEeprom(addr+offs,uintArr[i]);
  }
  return true;
}

//Reads array of uint16 values from EEPROM at address.
// addr:  EEPROM address.
// uintArr:  Array to receive uint16 values.
// maxArrCount:  Maximum number of entries for 'uintArr[]'.
// Returns number of entries read and stored in 'uintArr[]'.
int readUint16ArrayFromEeprom(int addr, uint16_t *uintArr, int maxArrCount)
{
  const uint16_t wordVal = readWordFromEeprom(addr);
  if((int)wordVal <= 0 || wordVal == (uint16_t)0xFFFF ||
                                                 (int)wordVal > maxArrCount)
  {  //length value out of bound or field not initialized
    return 0;
  }
  int offs = 0;
  for(int i=0; i<(int)wordVal; ++i)
  {  //for each entry; read from EEPROM location
    offs += 2;
    uintArr[i] = readWordFromEeprom(addr+offs);
  }
  return (int)wordVal;
}

//Interrupt-service routine that tracks the D2 input pin.
void d2InterruptRoutine()
{
  const byte newState = digitalRead(2);
  if(trackedD2InputState == HIGH && newState == LOW)
    ++triggerD2LiveCounter;    //if high-to-low transition then change value
  trackedD2InputState = newState;           //save current state
}

//Installs the interrupt-service routine for the D2 input pin.
void installD2InterruptRoutine()
{
  trackedD2InputState = digitalRead(2);     //start with initial state
  attachInterrupt(digitalPinToInterrupt(2),d2InterruptRoutine,CHANGE);
}

//Uninstalls the interrupt-service routine for the D2 input pin.
void uninstallD2InterruptRoutine()
{
  detachInterrupt(digitalPinToInterrupt(2));
}

//Returns the current state of the D2 input pin (as tracked by the ISR).
// Returns:  LOW or HIGH.
byte getD2InputCurrentState()
{
  return trackedD2InputState;
}

//Returns true if the D2 input pin has had a high-to-low transition since
// the last time this function was called.
boolean getD2InputTriggeredFlag()
{
  static byte triggerD2TrackCounter = 0;

  const byte liveVal = triggerD2LiveCounter;     //fetch value (thread safe)
  if(liveVal == triggerD2TrackCounter)      //if no change then
    return false;                           //return false no trigger
  triggerD2TrackCounter = liveVal;          //save new value for next time
  return true;
}

//Interrupt-service routine that tracks the D3 input pin.
void d3InterruptRoutine()
{
  const byte newState = digitalRead(3);
  if(trackedD3InputState == HIGH && newState == LOW)
    ++triggerD3LiveCounter;    //if high-to-low transition then change value
  trackedD3InputState = newState;           //save current state
}

//Installs the interrupt-service routine for the D3 input pin.
void installD3InterruptRoutine()
{
  trackedD3InputState = digitalRead(3);     //start with initial state
  attachInterrupt(digitalPinToInterrupt(3),d3InterruptRoutine,CHANGE);
}

//Uninstalls the interrupt-service routine for the D3 input pin.
void uninstallD3InterruptRoutine()
{
  detachInterrupt(digitalPinToInterrupt(3));
}

//Returns the current state of the D3 input pin (as tracked by the ISR).
// Returns:  LOW or HIGH.
byte getD3InputCurrentState()
{
  return trackedD3InputState;
}

//Returns true if the D3 input pin has had a high-to-low transition since
// the last time this function was called.
boolean getD3InputTriggeredFlag()
{
  static byte triggerD3TrackCounter = 0;

  const byte liveVal = triggerD3LiveCounter;     //fetch value (thread safe)
  if(liveVal == triggerD3TrackCounter)      //if no change then
    return false;                           //return false no trigger
  triggerD3TrackCounter = liveVal;          //save new value for next time
  return true;
}
