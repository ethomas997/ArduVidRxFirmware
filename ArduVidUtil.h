//ArduVidUtil.h:  Header file for ArduVidUtil.cpp functions.
//
//  2/18/2017 -- [ET]
//

#ifndef ARDUVIDUTIL_H_
#define ARDUVIDUTIL_H_

#define RECV_BUFSIZ 512                //serial-input buffer size

#define KEY_CR ((uint8_t)13)           //keyboard input codes
#define KEY_LF ((uint8_t)10)
#define KEY_BACKSP ((uint8_t)8)
#define KEY_DEL ((uint8_t)127)
#define KEY_ESC ((uint8_t)27)
#define KEY_ESCNXT ((uint8_t)91)
#define KEY_UP ((uint8_t)65)
#define KEY_DOWN ((uint8_t)66)
#define KEY_LEFT ((uint8_t)68)
#define KEY_RIGHT ((uint8_t)67)
#define KEY_HOME ((uint8_t)49)
#define KEY_END ((uint8_t)52)
#define KEY_PGUP ((uint8_t)53)
#define KEY_PGDN ((uint8_t)54)
#define KEYSEQ_ESC (((uint16_t)KEY_ESC)<<(uint16_t)8|KEY_ESCNXT)
#define KEY_REPORT ((uint8_t)'~')      //special report-RSSI key

#define CMD_KEY_UP 'U'       //command aliases for keyboard cursor keys
#define CMD_KEY_DOWN 'D'
#define CMD_KEY_LEFT 'P'
#define CMD_KEY_RIGHT 'N'
#define CMD_KEY_HOME 'S'
#define CMD_KEY_END 'M'

#define SERIAL_PROMPT_CHAR '>'         //prompt char for serial input
#define SERIAL_LIGNORE_CHAR ' '        //ignore line if begins with this

char *getNextSerialLine();
void flushSerialInputLines();
boolean getSerialInputAvailflag();
void setSerialInputPromptFlag();
void clearSerialInputPromptFlag();
boolean getDoReportRssiFlag();
char getLastCommandChar();
void clearLastCommandChar();
boolean convStrToInt(const char *str, int *pIntVal);
int removeValueFromArray(uint16_t *uintArr, int arrCount, uint16_t rmVal);
void showUint16ArrayList(const uint16_t intArr[], int intArrCount);
void doSoftwareReset();
void writeByteToEeprom(int addr, uint8_t val);
uint8_t readByteFromEeprom(int addr);
void writeWordToEeprom(int addr, uint16_t val);
uint16_t readWordFromEeprom(int addr);
void writeStringToEeprom(int addr, const char *str, int fieldLen);
int readStringFromEeprom(int addr, char *outStr, int fieldLen);
boolean isStringFromEepromEmpty(int addr);
int showStringFromEeprom(int addr, int fieldLen);
boolean writeUint16ArrayToEeprom(int addr, int fieldLen,
                                           uint16_t *uintArr, int arrCount);
int readUint16ArrayFromEeprom(int addr, uint16_t *uintArr, int maxArrCount);
void installD2InterruptRoutine();
void uninstallD2InterruptRoutine();
byte getD2InputCurrentState();
boolean getD2InputTriggeredFlag();
void installD3InterruptRoutine();
void uninstallD3InterruptRoutine();
byte getD3InputCurrentState();
boolean getD3InputTriggeredFlag();

extern boolean serialEchoFlag;

#endif /* ARDUVIDUTIL_H_ */
