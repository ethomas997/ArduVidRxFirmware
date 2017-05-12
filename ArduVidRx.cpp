//ArduVidRx.cpp:  Control program for FPV video receiver via Arduino.
//
// 12/17/2016 -- [ET]  Version 1.3:  Initial version.
//  1/26/2017 -- [ET]  Version 1.4:  Added 'XR' command.
//  4/22/2017 -- [ET]  Version 1.5:  Added automatic RSSI calibration and
//                     'XA' command; added code-configurable pullups for
//                     unused input lines.
//  4/25/2017 -- [ET]  Version 1.6:  Modified 'processAutoRssiCalValue()'
//                     to detect low-RSSI values on freqs between 5640 and
//                     5950 and to prevent min-raw-RSSI scale value from
//                     getting too low.
//  4/28/2017 -- [ET]  Version 1.7:  Added RSSI_SEC_PIN (so A6 can be
//                     used for RSSI if no signal on A7); modified
//                     'processAutoRssiCalValue()' to handle lower
//                     ranges of raw-RSSI values; added optional "defaults"
//                     parameter to 'XZ'; added EEPROM-integrity check;
//                     added XT command (with default of 35 ms).
//  4/29/2017 -- [ET]  Version 1.71:  Added NODISP_PRIVSEL_PIN and
//                     and NODISP_SECVSEL_PIN options.
//  5/11/2017 -- [ET]  Version 1.8:  Modified so response of 'L' query
//                     with empty list and serial-echo off is "0".
//

//Global arrays:
//
//listFreqsMHzArr[]:  List of frequencies (in MHz) entered via 'L' command.
//
//scanRssiValuesArr[]:  RSSI values (read via scan) for each channel; each
// entry corresponds to a frequency in the Rx5808Fns 'channelFreqTable[]'
// or a frequency in 'listFreqsMHzArr[]' (if entered).
//
//idxSortedByRssiArr[]:  List of channel-index values sorted by RSSI values
// (in 'scanRssiValuesArr[]') in descending order.  If 'listFreqsMHzArr[]'
// values are entered then the index values are for 'listFreqsMHzArr[]'.
//
//idxSortedSelectedArr:  List of channel-index values selected from the
// 'idxSortedByRssiArr[]' array by squelching frequencies adjacent to those
// already loaded.  If 'listFreqsMHzArr[]' values are entered then they
// are copied in as-is.

#include <Arduino.h>
#include "Config.h"
#include "ArduVidUtil.h"
#include "Rx5808Fns.h"
#include "Display7Seg.h"
#include "FreqListPresets.h"

#define PROG_NAME_STR "ArduVidRx"
#define PROG_VERSION_STR "1.8"
#define LISTFREQMHZ_ARR_SIZE 80   //size for 'listFreqsMHzArr[]' array

#define EEPROM_ADRW_FREQ 0        //address for freq value in EEPROM (word)
#define EEPROM_ADRB_BTNMODE 2     //address for button mode in EEPROM (byte)
#define EEPROM_ADRB_AUTOCAL 3     //address for auto RSSI calibration (byte)
#define EEPROM_ADRW_RSSIMIN 4     //address for RSSI-scaling min in EEPROM
#define EEPROM_ADRW_RSSIMAX 6     //address for RSSI-scaling max in EEPROM
#define EEPROM_ADRW_CHECKWORD 8   //address for integrity-check value EEPROM
#define EEPROM_ADRB_MINTUNEMS 10  //address for RX5808 min-tune time (byte)
#define EEPROM_ADRS_UNITID 30     //address for unit ID in EEPROM (string)
#define EEPROM_FLEN_UNITID 20     //field length for unit ID in EEPROM
#define EEPROM_ADRA_FREQLIST 64   //address for freq list in EEPROM (array)
#define EEPROM_FLEN_FREQLIST 84   //field length (in bytes) for freq list
                                  //total length (in bytes) of EEPROM used:
#define EEPROM_USED_DATASIZE (EEPROM_ADRA_FREQLIST+EEPROM_FLEN_FREQLIST)
#define EEPROM_CHECK_VALUE 0x5242 //EEPROM integrity-check value

    //setup flags for button-interrupt usage depending on pin assignments:
#define BUTTONPINS_UP2DOWN3_FLAG (UP_BUTTON_PIN == 2 && DOWN_BUTTON_PIN == 3)
#define BUTTONPINS_UP3DOWN2_FLAG (UP_BUTTON_PIN == 3 && DOWN_BUTTON_PIN == 2)
#define BUTTONPINS_USEINTERRUPT_FLAG (BUTTONS_ENABLED_FLAG && \
                     (BUTTONPINS_UP2DOWN3_FLAG || BUTTONPINS_UP3DOWN2_FLAG))

uint16_t currentTunerFreqMhzOrCode = 0;
uint16_t currentTunerFreqInMhz = 0;
boolean contRssiOutFlag = false;
boolean contRssiListFlag = false;
uint16_t contRssiPrevFreqVal = 0;
boolean lastShowCurRssiListFlag = false;
boolean monitorModeNextFlag = false;
boolean displayRssiEnabledFlag = false;
int sessionDefMinRssiLevel = DEF_MIN_RSSI_LEVEL;
char itoaBuff[20];
uint16_t listFreqsMHzArr[LISTFREQMHZ_ARR_SIZE];    //values via 'L' command
uint8_t scanRssiValuesArr[LISTFREQMHZ_ARR_SIZE];   //RSSI vals for all chans
uint8_t idxSortedByRssiArr[LISTFREQMHZ_ARR_SIZE];  //indices sorted by RSSI
uint8_t idxSortedSelectedArr[CHANNEL_MAX_INDEX+1];
int listFreqsMHzArrCount = 0;
int idxSortedSelArrCount = 0;
int nextTuneChannelIndex = -1;
unsigned long lastNextTuneScanTime = 0;
unsigned long monitorModeNextChanTime = 0;
int monitorModeIntervalSecs = DEF_MONITOR_INTERVAL_SECS;
unsigned long rssiOutSamplingAvgrTotal = 0;
byte rssiOutSamplingAvgrCounter = 0;
unsigned long delayedSaveFreqToEepromTime = 0;
boolean delayedSaveFreqToEepromFlag = false;
uint16_t lastEepromFreqInMhzOrCode = 0;
byte buttonsFunctionModeValue = 0;
boolean autoRssiCalibEnabledFlag = true;
byte autoRssiCalibCounterValue = 0;
unsigned long autoRssiCalibMarkedTime = 0;
boolean autoRssiCalibShowOutputFlag = false;
#if DISP7SEG_ENABLED_FLAG
boolean displayConnectedFlag = true;
#else
const boolean displayConnectedFlag = false;
#endif

boolean processExtraCommand(const char *cmdStr);
void showHelpInformation();
void showExtraHelpInformation();
void showListCmdHelpInformation();
void showFrequencyTable();
void showRevisionInfo(boolean dispInfoFlag);
void processTuneCommand(const char *valueStr);
void showCurrentFreqency();
void processFreqsMHzList(const char *listStr);
void showFreqsMHzList();
void processShowRssiCmd(const char *valueStr, boolean contFlag);
uint16_t showCurrentRssi(boolean showListFlag, boolean showChanFlag);
boolean repeatShowCurrentRssi();
void processOneMHzCommand(boolean upFlag, boolean serialOutFlag);
void processIncFreqCodeCommand(boolean bandFlag, boolean upFlag,
                                                        boolean buttonFlag);
void processAutoScanAndTuneCommand(const char *valueStr);
void autoScanTuneNextChan(const char *valueStr, boolean scanForwardFlag,
                                                boolean rescanOnSingleFlag);
void processMonitorModeCommand(const char *valueStr);
void monitorAutoTuneNextChan();
boolean scanChannelsAndReport(int minRssiLevel, int fallbackRssiLevel,
      boolean inclAllFlag, boolean restoreFreqFlag, boolean showOutputFlag);
boolean processScanChannelsCommand(const char *valueStr, boolean inclAllFlag);
void scanChansGetRssiValues(boolean includeLBandFlag, boolean inclAllFlag,
                           boolean restoreFreqFlag, boolean showOutputFlag);
void fullScanShowRssiValues();
void processSerialEchoCommand(const char *valueStr);
void processRawRssiMinMaxCommand(const char *valueStr);
void processEnableAutoRssiCalibCmd(const char *valueStr);
void processMinTuneTimeCommand(const char *valueStr);
void processMinRssiCommand(const char *valueStr);
void processMonitorIntervalCmd(const char *valueStr);
void processUnitIdCommand(const char *valueStr);
void processSoftRebootCommand(const char *valueStr);
void processShowFreqPresetListCmd(const char *valueStr);
void processListTranslateInfoCmd(const char *listStr);
void loadIdxSortedByRssiArr(boolean inclAllFlag);
int loadIdxSortedSelectedArr();
void processShowInputsCmd(const char *listStr);
void showDebugInputs();
void checkReportTableValues();
void setCurrentFreqByMhzOrCode(uint16_t freqMhzOrCode);
uint16_t getCurrentFreqInMhz();
uint16_t getCurrentFreqCodeWord();
void setTunerChannelToFreq(uint16_t freqInMhz);
void updateRssiOutput();
void clearRssiOutput();
void updateRssiOutValue(uint16_t rssiVal);
void scheduleDelayedSaveFreqToEeprom(int secs);
void saveCurrentFreqToEeprom();
void setChanToFreqValFromEeprom();
void saveButtonModeToEeprom(byte modeVal);
byte loadButtonModeFromEeprom();
void saveRssiMinMaxValsToEeprom();
void loadRssiMinMaxValsFromEeprom();
void saveAutoRssiCalFlagToEeprom(boolean flagVal);
boolean loadAutoRssiCalFlagFromEeprom();
void saveMinTuneTimeMsToEeprom(byte timeVal);
byte loadMinTuneTimeMsFromEeprom();
void saveUnitIdToEeprom(const char *str);
boolean isUnitIdFromEepromEmpty();
void showUnitIdFromEeprom();
void saveListFreqsMHzArrToEeprom();
void loadListFreqsMHzArrFromEeprom();
void setEepromToDefaultsValues();
void checkEepromIntegrity();
void updateActivityIndicator(boolean activityFlag);
uint16_t readRssiValue();
void processAutoRssiCalValue(uint16_t rawVal);
#if BUTTONS_ENABLED_FLAG
void processButtonModeCommand(const char *valueStr);
byte fetchButtonsTriggerState();
char *processButtonInputs(boolean bEnabledFlag);
#endif
#if DISP7SEG_ENABLED_FLAG
void processWriteDisplayCmd(const char *valueStr);
void showProgramVersionOnDisplay();
void showTunerChannelOnDisplay();
void showNumericValueOnDisplay(uint16_t dispVal, boolean leftDpFlag,
                                       boolean rightDpFlag, int dispTimeMs);
void showRssiValueOnDisplay(uint16_t rssiVal);
#if BUTTONS_ENABLED_FLAG
void showButtonModeOnDisplay(byte bModeVal, int dispTimeMs);
#endif  //BUTTONS_ENABLED_FLAG
#endif  //DISP7SEG_ENABLED_FLAG


// SETUP ----------------------------------------------------------------------------
void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  checkEepromIntegrity();    //check EEPROM; reset to defaults if needed
#if DISP7SEG_ENABLED_FLAG    //detect if display is actually wired in:
  displayConnectedFlag = disp7SegTestDisplayConnected();
  if(displayConnectedFlag)
  {  //display is actually wired in
    disp7SegSetup();         //do hardware setup for 7-segment displays
    showProgramVersionOnDisplay();
  }
  else                                      //if no display then
    pinMode(NODISP_ACTIVITY_PIN,OUTPUT);    //enable activity-indicator pin
#else
  pinMode(NODISP_ACTIVITY_PIN,OUTPUT);
#endif  //DISP7SEG_ENABLED_FLAG
#if BUTTONS_ENABLED_FLAG
  buttonsFunctionModeValue = loadButtonModeFromEeprom();
  pinMode(UP_BUTTON_PIN,INPUT_PULLUP);           //setup button inputs
  pinMode(DOWN_BUTTON_PIN,INPUT_PULLUP);
#if BUTTONPINS_USEINTERRUPT_FLAG
  installD2InterruptRoutine();
  installD3InterruptRoutine();
#endif  //BUTTONPINS_USEINTERRUPT_FLAG
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
    showButtonModeOnDisplay(buttonsFunctionModeValue,1000);
#endif  //DISP7SEG_ENABLED_FLAG
#endif  //BUTTONS_ENABLED_FLAG
#ifdef PULLUP_1_PIN                         //if setup then configure
  pinMode(PULLUP_1_PIN,INPUT_PULLUP);       // unused inputs to have pullups
#endif
#ifdef PULLUP_2_PIN
  pinMode(PULLUP_2_PIN,INPUT_PULLUP);
#endif
  serialEchoFlag = true;
  setRx5808MinTuneTimeMs(loadMinTuneTimeMsFromEeprom());
  rx5808setup();
  loadRssiMinMaxValsFromEeprom();      //load RSSI-scaling values from EEPROM
                                       //load auto calib flag from EEPROM:
  autoRssiCalibEnabledFlag = loadAutoRssiCalFlagFromEeprom();
  loadListFreqsMHzArrFromEeprom();     //load array of 'L'-command freqs
    //set tuner to freq value from EEPROM (or default if never saved):
  setChanToFreqValFromEeprom();
  if(!displayConnectedFlag)
  {  //7-segment display not connected; set pins to select video output
#ifdef NODISP_PRIVSEL_PIN              // (needed for diversity boards)
    pinMode(NODISP_PRIVSEL_PIN,OUTPUT);
    digitalWrite(NODISP_PRIVSEL_PIN,
                                  isPriRx5808RssiInPinInUse() ? HIGH : LOW);
#endif
#ifdef NODISP_SECVSEL_PIN
    pinMode(NODISP_SECVSEL_PIN,OUTPUT);
    digitalWrite(NODISP_SECVSEL_PIN,
                                  isPriRx5808RssiInPinInUse() ? LOW : HIGH);
#endif
  }
  Serial.println();
  showRevisionInfo(false);
  showCurrentFreqency();
  if(listFreqsMHzArrCount > 0)
  {  //frequency list (via 'L' command) not empty
    Serial.print(F(" Using freq list: "));
    showFreqsMHzList();
  }
}

//Performs shutdown-cleanup actions (disconnects interrupts).
void doShutdownCleanup()
{
#if BUTTONPINS_USEINTERRUPT_FLAG
  uninstallD2InterruptRoutine();
  uninstallD3InterruptRoutine();
#endif
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
    disp7SegShutdown();
#endif
}

// LOOP ----------------------------------------------------------------------------
void loop()
{
  if(delayedSaveFreqToEepromFlag && millis() > delayedSaveFreqToEepromTime)
  {  //save freq to EEPROM scheduled and time reached
    delayedSaveFreqToEepromFlag = false;
    saveCurrentFreqToEeprom();
  }
         //check for next line of serial input:
  const char *nextSerialLineStr = getNextSerialLine();
  const boolean serialAvailFlag =      //serial chars or full line available
                   (getSerialInputAvailflag() || nextSerialLineStr != NULL);
         //if report-RSSI char was received (and not continuous
         // RSSI output in progess) then show RSSI/channel:
  if(getDoReportRssiFlag() && !contRssiOutFlag)
    showCurrentRssi(false,true);

  const char *cmdStr;

#if BUTTONS_ENABLED_FLAG
         //process button inputs (disabled if serial in or continuous RSSI):
  cmdStr = processButtonInputs(!(serialAvailFlag || contRssiOutFlag));
  if(cmdStr != NULL)
  {  //new command via button action
    if(monitorModeNextFlag)
    {  //auto-tune-monitor mode is in progress then
      monitorModeNextFlag = false;     //stop monitor mode
      cmdStr = NULL;                   //discard command
    }
    else if(serialEchoFlag)
    {  //serial echo enabled
      Serial.println(cmdStr);          //show command
    }
    setSerialInputPromptFlag();        //setup to show prompt later
         //clear any previous command (so can't be invoked via <Enter> key):
    clearLastCommandChar();
  }
#else
  cmdStr = NULL;
#endif

  if(contRssiOutFlag)
  {  //continuous RSSI output enabled
    if(!serialAvailFlag)
    {  //no serial port input received
      const uint16_t rVal = showCurrentRssi(contRssiListFlag,false);
      updateRssiOutValue(rVal);        //update analog-RSSI output
      if(!displayConnectedFlag)             //if no display then
        updateActivityIndicator(true);      //indicate "extra" activity
      return;
    }
    else
    {  //serial port input detected
      contRssiOutFlag = false;                        //stop output
      monitorModeNextFlag = false;                    //make sure both stopped
      if(contRssiPrevFreqVal > (uint16_t)0)           //if saved then
        setTunerChannelToFreq(contRssiPrevFreqVal);   //restore tuner freq
      clearRssiOutput();
    }
  }
  else if(monitorModeNextFlag)
  {  //auto-tune-monitor mode enabled
    if(!serialAvailFlag)
    {  //no serial port input received
      monitorAutoTuneNextChan();
      updateRssiOutput();              //update analog-RSSI output
      return;
    }
    else
    {  //serial port input detected
      monitorModeNextFlag = false;               //stop output
      contRssiOutFlag = false;                   //make sure both stopped
      clearRssiOutput();
    }
  }
  else   //neither output mode enabled
    updateRssiOutput();           //update analog-RSSI output

  const boolean buttonInFlag = (cmdStr != NULL);
  int sLen;        //if no input via buttons then check serial input:
  if(((buttonInFlag && (sLen=strlen(cmdStr)) > 0)) ||
     ((cmdStr=nextSerialLineStr) != NULL && ((sLen=strlen(cmdStr)) == 0 ||
     (cmdStr[0] != SERIAL_PROMPT_CHAR && cmdStr[0] != SERIAL_LIGNORE_CHAR &&
                                strncasecmp(cmdStr,PROG_NAME_STR,4) != 0))))
  {  //line of input data was received and doesn't begin with prompt,
     // space or sign-on string that may be received if slave receiver
              //if display not connected then always show "extra" activity:
    if(!displayConnectedFlag)
      updateActivityIndicator(true);
    boolean displayActFlag = false;    //set below for activity indicator
    int p = 0;
    while(p < sLen && cmdStr[p] == ' ')
      ++p;              //ignore any leading spaces
    if(p < sLen)
    {  //command not empty
      const char cmdChar = (char)toupper(cmdStr[p]);
      switch(cmdChar)
      {
        case 'T':       //tune receiver to given MHz value
          processTuneCommand(&cmdStr[p+1]);
          displayActFlag = true;            //indicate activity on display
          break;
        case 'A':       //auto-scan and tune to highest-RSSI channel
          processAutoScanAndTuneCommand(&cmdStr[p+1]);
          break;
        case 'N':       //auto-scan and tune to next channel
          autoScanTuneNextChan(&cmdStr[p+1],true,true);
          break;
        case 'P':       //auto-scan and tune to previous channel
          autoScanTuneNextChan(&cmdStr[p+1],false,true);
          break;
        case 'M':       //auto-scan and monitor channels
          processMonitorModeCommand(&cmdStr[p+1]);
          break;
        case 'S':       //scan and report channels with highest RSSI
          processScanChannelsCommand(&cmdStr[p+1],false);
          break;
        case 'F':       //scan and report RSSI for full set of channels
          processScanChannelsCommand(&cmdStr[p+1],true);
          break;
        case 'L':       //list of freq (MHz) values to be scanned
          processFreqsMHzList(&cmdStr[p+1]);
          displayActFlag = true;            //indicate activity on display
          break;
        case 'R':       //read RSSI
          processShowRssiCmd(&cmdStr[p+1],false);
          displayActFlag = true;            //indicate activity on display
          break;
        case 'O':       //continuous RSSI display
          processShowRssiCmd(&cmdStr[p+1],true);
          break;
        case 'U':       //increase turned frequency by one MHz
          processOneMHzCommand(true,serialEchoFlag);
          break;
        case 'D':       //decrease turned frequency by one MHz
          processOneMHzCommand(false,serialEchoFlag);
          break;
        case 'B':       //increment band on tuned-frequency code
          processIncFreqCodeCommand(true,true,false);
          break;
        case 'C':       //increment channel on tuned-frequency code
          processIncFreqCodeCommand(false,true,false);
          break;
        case 'G':       //show raw debug inputs values
          processShowInputsCmd(&cmdStr[p+1]);
          displayActFlag = true;            //indicate activity on display
          break;
#if DISP7SEG_ENABLED_FLAG
        case '#':       //toggle showing live RSSI on display
          if(displayConnectedFlag)
          {  //display is actually wired in
            if(!displayRssiEnabledFlag)
              displayRssiEnabledFlag = true;
            else
            {  //disable showing
              displayRssiEnabledFlag = false;
              disp7SegClearOvrDisplay();    //clear RSSI value immediately
            }
          }
          break;
#endif
#if BUTTONS_ENABLED_FLAG
        case '=':       //set/show button mode value
          processButtonModeCommand(&cmdStr[p+1]);
          break;
#endif
        case 'E':       //serial echo on/off or echo text
          processSerialEchoCommand(&cmdStr[p+1]);
          displayActFlag = true;            //indicate activity on display
          break;
        case 'V':       //show program-version information
          showRevisionInfo(true);
          displayActFlag = true;            //indicate activity on display
          break;
        case 'H':       //show help screen
        case '?':
          showHelpInformation();
          displayActFlag = true;            //indicate activity on display
          break;
        case 'I':       //show frequency-information screen
          showFrequencyTable();
          displayActFlag = true;            //indicate activity on display
          break;
        case 'X':       //process "extra" command
          displayActFlag = processExtraCommand(&cmdStr[p+1]);
          break;
        default:
          Serial.print(F(" Unrecognized command:  "));
          Serial.print(&cmdStr[p]);
          Serial.println(F("  [Enter H for help]"));
          displayActFlag = true;            //indicate activity on display
      }
    }
    else //received command line is empty,
    {    // repeat last command (if one of those below)
      const char lastCommandChar = getLastCommandChar();
      if(lastCommandChar == 'R')
      {
        if(!repeatShowCurrentRssi())        //if not 'L' list then
          displayActFlag = true;            //indicate activity on display
      }
      else if(lastCommandChar == 'N')
        autoScanTuneNextChan("",true,true);
      else if(lastCommandChar == 'P')
        autoScanTuneNextChan("",false,true);
      else if(lastCommandChar == 'B')
        processIncFreqCodeCommand(true,true,false);
      else if(lastCommandChar == 'C')
        processIncFreqCodeCommand(false,true,false);
      else if(lastCommandChar == 'G')
      {
        processShowInputsCmd("");
        displayActFlag = true;              //indicate activity on display
      }
      else  //empty command line and no repeat command
        displayActFlag = true;              //indicate activity on display
    }
         //if display connected and flag was set then show "extra" activity:
    if(displayConnectedFlag && displayActFlag && !buttonInFlag)
      updateActivityIndicator(true);
  }
  else  //no input-command data received
    updateActivityIndicator(false);         //indicate normal activity
}

//Processes "extra" (X) command.
// Returns true if activity should be indicated on display; false if not.
boolean processExtraCommand(const char *cmdStr)
{
  boolean retFlag = true;
  const int sLen = strlen(cmdStr);
  int p = 0;
  while(cmdStr[p] == ' ' && ++p < sLen);    //ignore any leading spaces
  if(p >= sLen)
  {  //no parameters given
    showExtraHelpInformation();
    return retFlag;
  }
  const char cmdChar = (char)toupper(cmdStr[p]);
  switch(cmdChar)
  {
    case 'J':      //set or show raw-RSSI-scaling values
      processRawRssiMinMaxCommand(&cmdStr[p+1]);
      break;
    case 'A':      //disable/enable/restart auto RSSI calibration
      processEnableAutoRssiCalibCmd(&cmdStr[p+1]);
      break;
    case 'T':      //set or show RX5808 min-tune time (ms)
      processMinTuneTimeCommand(&cmdStr[p+1]);
      break;
    case 'M':      //set or show minimum-RSSI value for scans
      processMinRssiCommand(&cmdStr[p+1]);
      break;
    case 'I':      //set or show monitor-mode interval
      processMonitorIntervalCmd(&cmdStr[p+1]);
      break;
    case 'U':      //set or show Unit-ID string
      processUnitIdCommand(&cmdStr[p+1]);
      break;
    case 'R':      //read and show RSSI (with channel info)
      showCurrentRssi(false,true);
      retFlag = !displayConnectedFlag;      //indicator if no display
      break;
    case 'L':      //show frequency list for preset name
      processShowFreqPresetListCmd(&cmdStr[p+1]);
      break;
    case 'P':      //show all frequency-list presets
      freqListPresetShowAllSets();
      break;
    case 'B':      //decrement band on tuned-frequency code
      processIncFreqCodeCommand(true,false,false);
      break;
    case 'C':      //decrement channel on tuned-frequency code
      processIncFreqCodeCommand(false,false,false);
      break;
    case 'F':      //perform and report full scan of all freqs
      fullScanShowRssiValues();
      break;
    case 'X':      //show index values for frequencies (devel)
      processListTranslateInfoCmd(&cmdStr[p+1]);
      break;
#if DISP7SEG_ENABLED_FLAG
    case 'D':      //show given chars on 7-segment displays
      if(displayConnectedFlag)
      {
        processWriteDisplayCmd(&cmdStr[p+1]);
        retFlag = false;     //no indicator (interferes with display)
      }
      break;
#endif
    case 'K':      //check/report calc vs table values (debug)
      checkReportTableValues();
      break;
    case 'Z':      //soft reboot
      processSoftRebootCommand(&cmdStr[p+1]);
      break;
    case 'H':      //show extra help screen
    case '?':
      showExtraHelpInformation();
      break;
    default:
      Serial.print(F(" Unrecognized 'extra' command:  "));
      Serial.print(&cmdStr[p]);
      Serial.println(F("  [Enter XH for help]"));
  }
  return retFlag;
}

//Displays help screen.
void showHelpInformation()
{
  showRevisionInfo(false);
  Serial.println(F(" Commands:"));
  Serial.println(F("  T [freq]    : Tune receiver to given MHz or XX code"));
  Serial.println(F("  A           : Auto-scan and tune to highest-RSSI channel"));
  Serial.println(F("  N [minRSSI] : Auto-scan and tune to next channel"));
  Serial.println(F("  P [minRSSI] : Auto-scan and tune to previous channel"));
  Serial.println(F("  M [seconds] : Auto-scan and monitor channels"));
  Serial.println(F("  S [minRSSI] : Scan and report channels with highest RSSI"));
  Serial.println(F("  F [minRSSI] : Scan and report RSSI for full set of channels"));
  Serial.println(F("  L [list]    : List of freqs of interest (LH for help)"));
  Serial.println(F("  R           : Read RSSI for current channel (RL for 'L' freqs)"));
  Serial.println(F("  O           : Continuous RSSI display (OL for 'L' freqs)"));
  Serial.println(F("  U / D       : Change tuned frequency up/down by one MHz"));
  Serial.println(F("  B / C       : Increment band/channel on tuned-frequency code"));
  Serial.println(F("  X           : Extra commands (XH for help)"));
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
    Serial.println(F("  #           : Toggle showing live RSSI on display"));
#endif
#if BUTTONS_ENABLED_FLAG
  Serial.println(F("  =           : Set or show button mode value"));
#endif
  Serial.println(F("  V           : Show program-version information"));
  Serial.println(F("  I           : Show frequency-table information"));
  Serial.println(F("  H or ?      : Show help information"));
}

//Displays help screen for 'X' commands.
void showExtraHelpInformation()
{
  Serial.println(F(" Extra commands:"));
  Serial.println(F("  XJ [min,max]  : Set or show RSSI-scaling values"));
  Serial.println(F("  XJ default    : Set RSSI-scaling values to defaults"));
  Serial.println(F("  XA [0|1|R]    : Disable/enable/restart auto RSSI calib"));
  Serial.println(F("  XT [timeMs]   : Set or show RX5808 min-tune time (ms)"));
  Serial.println(F("  XM [minRSSI]  : Set or show minimum RSSI for scans"));
  Serial.println(F("  XI [seconds]  : Set or show monitor-mode interval"));
  Serial.println(F("  XU [text]     : Set or show Unit-ID string"));
  Serial.println(F("  XR or ~       : Read and show RSSI (with channel info)"));
  Serial.println(F("  XB / XC       : Decrement band/channel on tuned-freq code"));
  Serial.println(F("  XF            : Perform and report full scan of all freqs"));
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
    Serial.println(F("  XD [chars]    : Show given chars on display"));
#endif
  Serial.println(F("  XP            : Show all frequency-list presets"));
  Serial.println(F("  XL [name]     : Show frequency list for preset name"));
  Serial.println(F("  XX [list]     : Show index values for frequencies (devel)"));
  Serial.println(F("  XK            : Show frequency table values (devel)"));
  Serial.println(F("  XZ [defaults] : Perform soft program reboot"));
  Serial.println(F("  X, XH or X?   : Show extra help information"));
}

//Displays help screen for 'L' command.
void showListCmdHelpInformation()
{
  Serial.println(F(" Frequency-list command:"));
  Serial.println(F("  L [list]    : Set list of freq (MHz) values of interest"));
  Serial.println(F("  L           : Show list of freq (MHz) values of interest"));
  Serial.println(F("  L 0         : Clear list of freq values of interest"));
  Serial.println(F("  L +values   : Add values to current list"));
  Serial.println(F("  L -values   : Remove values from current list"));
  Serial.println(F("  L S         : Load list of freqs via RSSI scan"));
  Serial.println(F("  L H         : Show help information for 'L' command"));
  Serial.println(F(" When a list is entered, the frequencies in the list will be the only ones"));
  Serial.println(F(" scanned and selected by the 'A', 'S', 'N', 'P' and 'M' commands.  The 'RL'"));
  Serial.println(F(" and 'OL' commands will scan and display RSSI values for the frequencies in"));
  Serial.println(F(" the list.  Entering 'L 0' will clear the list.  The '+' and '-' operators"));
  Serial.println(F(" may be used to add and remove frequencies, and may be mixed together"));
  Serial.println(F(" (i.e., 'L +5740 -5905').  The 'L S' command will load the list with the"));
  Serial.println(F(" frequency set returned by the last scan ('S' command), or will perform a"));
  Serial.println(F(" scan and load the detected values.  Frequency-list-preset names may also"));
  Serial.println(F(" be used as parameters to the 'L' command (i.e., 'L IMD5').  Available"));
  Serial.println(F(" presets may be displayed via the 'XP' command."));
}

//Displays frequency table.
void showFrequencyTable()
{
  Serial.println(F(" Frequency Table    1     2     3     4     5     6     7     8"));
  Serial.println(F(" Frequency band A  5865  5845  5825  5805  5785  5765  5745  5725"));
  Serial.println(F(" Frequency band B  5733  5752  5771  5790  5809  5828  5847  5866"));
  Serial.println(F(" Frequency band E  5705  5685  5665  5645  5885  5905  5925  5945"));
  Serial.println(F(" Frequency band F  5740  5760  5780  5800  5820  5840  5860  5880"));
  Serial.println(F(" Frequency band R  5658  5695  5732  5769  5806  5843  5880  5917"));
  Serial.println(F(" Frequency band L  5362  5399  5436  5473  5510  5547  5584  5621"));
}

//Shows the "Unable to parse value" message via serial.
void showUnableToParseValueMsg()
{
  Serial.print(F(" Unable to parse value:  "));
}

//Shows program-version information.
// dispInfoFlag:  true to include information about 7-segment displays.
void showRevisionInfo(boolean dispInfoFlag)
{
  Serial.print(F(PROG_NAME_STR));
  Serial.print(F(" Version "));
  Serial.print(F(PROG_VERSION_STR));
  if(!isUnitIdFromEepromEmpty())
  {  //Unit-ID string not empty; show it
    Serial.print(F(", Unit ID: "));
    showUnitIdFromEeprom();
  }
  Serial.println();
  if(dispInfoFlag)
  {  //show display information
    Serial.print(F("  Display:  "));
#if DISP7SEG_ENABLED_FLAG
    if(displayConnectedFlag)
      Serial.print(F("Connected"));
    else
      Serial.print(F("Not connected"));
#else
    Serial.print(F("Not supported (disabled via build option)"));
#endif
  Serial.println();
  }
}

//Sets tuner frequency to given frequency (MHz) or frequency code word,
//  and sends status text to the serial port.
// freqInMHz:  Frequency value in MHz.
// codeVal:  Two-character frequency code packed into 2-byte word
//           (high byte is band character), or zero for none.
void doTuneToFreqMHzOrCode(uint16_t freqInMHz, uint16_t codeVal)
{
  uint16_t freqMhzOrCode;
  if(serialEchoFlag)
  {  //more output if echo enabled
    Serial.print(F(" Tuning to frequency "));
    Serial.print((int)freqInMHz);
    Serial.print(F("MHz"));
    if(codeVal > (uint16_t)0)
    {  //frequency-code value available
      freqMhzOrCode = codeVal;         //tune to code-word value
      Serial.print(" (");
      Serial.print((char)(codeVal >> (uint16_t)8));
      Serial.print((char)codeVal);
      Serial.print(')');
    }
    else
      freqMhzOrCode = freqInMHz;       //tune to freq in MHz
     Serial.println();
  }
  else        //setup to tune to code word (if available) or freq in MHz:
    freqMhzOrCode = (codeVal > (uint16_t)0) ? codeVal : freqInMHz;
  setTunerChannelToFreq(freqMhzOrCode);
}

//Processes the 'tune' command with the given frequency (MHz) string.
void processTuneCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen <= 0)
  {  //no parameter value; show current
    showCurrentFreqency();
    return;
  }
  int iVal;
  if(sLen > 2 && convStrToInt(valueStr,&iVal))
  {  //more than 2 characters and numeric value parsed OK
    if(iVal < MIN_CHANNEL_MHZ || iVal > MAX_CHANNEL_MHZ)
    {  //range check failed
      Serial.print(F(" Value out of range:  "));
      Serial.println(iVal);
      return;
    }
         //see if frequency corresponds to a frequency code:
    const uint16_t codeVal = freqInMhzToFreqCode((uint16_t)iVal,NULL);
         //tune and send status to serial port:
    doTuneToFreqMHzOrCode((uint16_t)iVal,codeVal);
         //save tuned freq to EEPROM 3 secs after last set:
    scheduleDelayedSaveFreqToEeprom(3);
    return;
  }
         //attempt to process as 2-character frequency code:
  const uint16_t codeVal = freqCodeStrToCodeWord(valueStr);
                   //get freq in MHz for code word:
  const uint16_t freqInMHz = freqCodeWordToFreqInMhz(codeVal);
  if(freqInMHz > (uint16_t)0)
  {  //2-character frequency code parsed and converted OK
         //tune and send status to serial port:
    doTuneToFreqMHzOrCode(freqInMHz,codeVal);
         //save tuned freq to EEPROM 3 secs after last set:
    scheduleDelayedSaveFreqToEeprom(3);
    return;
  }
    //parsing failed
  showUnableToParseValueMsg();
  Serial.println(valueStr);
}

//Displays the current frequency.
void showCurrentFreqency()
{
  if(serialEchoFlag)
  {  //more output if echo enabled
    Serial.print(F(" Current frequency is "));
    const uint16_t freqVal = getCurrentFreqInMhz();
    Serial.print((int)freqVal);
    Serial.print("MHz");
    const uint16_t codeVal = getCurrentFreqCodeWord();
    if(codeVal > (uint16_t)0)
    {  //frequency-code value available; show it
      Serial.print(" (");
      Serial.print((char)(codeVal >> (uint16_t)8));
      Serial.print((char)(codeVal & (uint16_t)0x7F));
      Serial.print(')');
    }
    Serial.println();
  }
  else
  {  //serial echo not enabled; show simple output
    Serial.print(' ');
    Serial.println((int)getCurrentFreqInMhz());
  }
}

//Processes the given list of frequency (MHz) values and enters them
// into the 'scanFreqsMHzArr[]' array.
void processFreqsMHzList(const char *listStr)
{
  const int sLen = strlen(listStr);
  int sPos = 0;
  while(listStr[sPos] == ' ' && ++sPos < sLen);  //ignore any leading spaces
  if(sPos >= sLen)
  {  //no parameters given
    showFreqsMHzList();           //show current list
    return;
  }
  int numItems;
  if(listStr[sPos] == 'H' || listStr[sPos] == 'h' || listStr[sPos] == '?')
  {
    showListCmdHelpInformation();
    return;
  }
  if((listStr[sPos] == 'S' || listStr[sPos] == 's') && sPos+1 == sLen)
  {  //use frequency values from band scan
    numItems = listFreqsMHzArrCount;   //save current count
    listFreqsMHzArrCount = 0;          //clear any existing entries
    if(idxSortedSelArrCount <= 0 ||
               millis() >= lastNextTuneScanTime + NEXT_CHAN_RESCANSECS*1000)
    {  //no freqs available from previous scan or too much time elapsed
      if(!scanChannelsAndReport(sessionDefMinRssiLevel,   //do band scan now
                          sessionDefMinRssiLevel,false,true,serialEchoFlag))
      {  //no channels with high enough RSSI found
        listFreqsMHzArrCount = numItems;    //keep existing list (if any)
        return;
      }
    }
    if(idxSortedSelArrCount <= 0)
      return;      //abort if no freqs available (shouldn't happen)
    for(numItems=0; numItems<idxSortedSelArrCount; ++numItems)
    {  //for each index value in array; fetch and copy frequency value
      listFreqsMHzArr[numItems] = getChannelFreqTableEntry(
                                            idxSortedSelectedArr[numItems]);
    }
    idxSortedSelArrCount = 0;  //idxSortedSelectedArr[] values no longer valid
  }
  else
  {  //don't use frequency values from band scan
    idxSortedSelArrCount = 0;  //idxSortedSelectedArr[] values no longer valid
    const int listStrLen = strlen(listStr);
    boolean plusFlag = false, minusFlag = false, firstFlag = true;
    int val, ePos, psetCount;
    char ch;
    numItems = listFreqsMHzArrCount;   //setup to append entries (for now)
    while(true)
    {  //for each item in list
      while(sPos < listStrLen)
      {  //scan through any +/-, whitespace or comma characters
        ch = listStr[sPos];
        if(ch == '+')             //handle any leading +/- sign
        {
          plusFlag = true;        //indicate '+' sign (only matters for 1st)
          minusFlag = false;      //set "adding" mode
        }
        else if(ch == '-')
          minusFlag = true;       //set "removing" mode
        else if(ch != ' ' && ch != '\t' && ch != ',')
          break;        //if not whitespace or comma then exit inner loop
        ++sPos;
      }
      if(sPos >= listStrLen)           //if end of input string then
        break;                         //exit loop
         //if first entry & no leading +/- then remove all current entries:
      if(firstFlag && !plusFlag && !minusFlag)
        numItems = 0;
      ePos = sPos;                //scan through entry characters
      while(ePos < listStrLen && (ch=listStr[++ePos]) != ' ' &&
                         ch != '\t' && ch != ',' && ch != '+' && ch != '-');
      if(convStrToInt(&listStr[sPos],&val))
      {  //successfully parsed numeric value
        if(val < MIN_CHANNEL_MHZ || val > MAX_CHANNEL_MHZ)
        {  //value out of range
          if(val != 0 || !firstFlag || plusFlag || minusFlag)
          {  //not leading zero value (for list clear)
            Serial.print(F(" Entered value out of range:  "));
            Serial.println(val);
          }
          break;
        }
        if(minusFlag)
        {  //removing value from list
          numItems = removeValueFromArray(
                                    listFreqsMHzArr,numItems,(uint16_t)val);
        }
        else
        {  //adding value to list
          numItems = removeValueFromArray(  //if already in list then remove
                                    listFreqsMHzArr,numItems,(uint16_t)val);
          if(numItems >= LISTFREQMHZ_ARR_SIZE)
          {
            Serial.println(F(" Too many values specified"));
            break;
          }
          listFreqsMHzArr[numItems++] = (uint16_t)val;
        }
      }
      else if((psetCount=freqListPresetLoadByName(&listStr[sPos],
             &listFreqsMHzArr[numItems],LISTFREQMHZ_ARR_SIZE-numItems)) > 0)
      {  //successfully parsed entry as frequency-preset name
        if(minusFlag)
        {  //removing values from list
          Serial.println(F(" Removal via freq-preset name not supported"));
          break;
        }
        else
        {  //adding values to list
          numItems += psetCount;  //increase list size by freq values added
          if(numItems >= LISTFREQMHZ_ARR_SIZE)
          {
            Serial.println(F(" Reached maximum list size"));
            break;
          }
        }
      }
      else
      {  //unable to parse entry as numeric or as frequency-preset name
        if(firstFlag)
        {  //this is the first entry
          Serial.print(F(" Error processing input:  "));
          Serial.print(&listStr[sPos]);
          if(serialEchoFlag)
            Serial.print(F("  [Enter LH for help]"));
          Serial.println();
          return;       //don't show list contents below
        }
           //not first entry
        Serial.print(F(" Error processing value(s):  "));
        Serial.print(&listStr[sPos]);
        if(serialEchoFlag)
          Serial.print(F("  [Enter LH for help]"));
        Serial.println();
        if(!serialEchoFlag)  //if serial echo off then
          return;            //don't follow error msg with numItems display
        break;
      }
      sPos = ePos;
      firstFlag = false;
    }
  }
  listFreqsMHzArrCount = numItems;
  saveListFreqsMHzArrToEeprom();       //store new list in EEPROM
  Serial.print(' ');    //start with leading space (so ignored by slave recvr)
  if(serialEchoFlag)
  {  //echo on; show list of frequencies entered
    if(numItems > 0)
    {
      Serial.print(numItems);
      Serial.print(F(" value"));
      if(numItems != 1)
        Serial.print('s');
      Serial.print(F(":  L"));
      showUint16ArrayList(listFreqsMHzArr,numItems);
      Serial.println();
    }
    else
      Serial.println(F("List is empty"));
  }
  else
    Serial.println(numItems);
}

//Shows the current list of frequency (MHz) values in the
// 'scanFreqsMHzArr[]' array.
void showFreqsMHzList()
{
  Serial.print(' ');    //start with leading space (so ignored by slave recvr)
  if(listFreqsMHzArrCount > 0)
  {
    showUint16ArrayList(listFreqsMHzArr,listFreqsMHzArrCount);
    Serial.println();
  }
  else
  {
    if(serialEchoFlag)
      Serial.println(F("List is empty"));
    else
      Serial.println('0');
  }
}

//Processes the show-RSSI command.  May have 'L' parameter to show
// RSSI values for frequencies entered via 'L' command.
// contFlag:  true if 'O' command (continuous output); false if 'R' command.
void processShowRssiCmd(const char *valueStr, boolean contFlag)
{
  const int sLen = strlen(valueStr);
  int p = 0;
  while(valueStr[p] == ' ' && ++p < sLen);  //ignore any leading spaces
  if(p >= sLen)
  {  //no parameter given
    if(contFlag)
    {  //'O' command
      contRssiOutFlag = true;     //start continuous-RSSI display
      contRssiListFlag = false;   //don't show for 'L' list
      contRssiPrevFreqVal = 0;    //don't need to restore frequency
      clearSerialInputPromptFlag();    //suppress '>' serial prompt
    }
    else  //'R' command
      showCurrentRssi(false,false);    //show RSSI for currently-tuned freq
    return;
  }
  if(valueStr[p] != 'L' && valueStr[p] != 'l')
  {  //parameter is not 'L'
    Serial.print(F(" Invalid parameter:  "));
    Serial.println(&valueStr[p]);
    return;
  }
  if(listFreqsMHzArrCount <= 0)
  {
    Serial.println(F(" Frequency list (via 'L' command) is empty"));
    return;
  }
         //frequency list was entered via 'L' command
  if(contFlag)
  {  //'OL' command
    contRssiOutFlag = true;     //start continuous-RSSI display
    contRssiListFlag = true;    //show for 'L' list
    contRssiPrevFreqVal = currentTunerFreqMhzOrCode;     //save tuned freq
    clearSerialInputPromptFlag();      //suppress '>' serial prompt
  }
  else
  {  //'RL' command
    const uint16_t prevFreqVal = currentTunerFreqMhzOrCode;
    showCurrentRssi(true,false);            //show RSSI for list
    setTunerChannelToFreq(prevFreqVal);     //restore tuner frequency
  }
}

//Shows the current RSSI value(s).
// showListFlag:  true to show RSSI values for frequencies entered via
//                'L' command.
// showChanFlag:  true to include channel info (only applies when
//                'showListFlag==false').
// Returns:  The current RSSI value, or zero if more than one was shown.
uint16_t showCurrentRssi(boolean showListFlag, boolean showChanFlag)
{
  lastShowCurRssiListFlag = showListFlag;   //save for 'repeatShowCurrentRssi()'
  if(showListFlag)
  {  //showing RSSI values for frequencies entered via 'L' command
    if(listFreqsMHzArrCount <= 0)
      return (uint16_t)0;              //if no list then abort
    Serial.print(' ');
    int i = 0;
    uint16_t wordVal;
    while(true)
    {  //for each frequency value in list
      if(serialEchoFlag)
      {  //show extra info
        Serial.print(listFreqsMHzArr[i]);
        Serial.print('=');
      }
      const uint16_t freqVal = listFreqsMHzArr[i];
      setCurrentFreqByMhzOrCode(freqVal);
#if DISP7SEG_ENABLED_FLAG
      if(displayConnectedFlag)
      {  //display enabled; show freq code for freq value (if any)
        if((wordVal=freqInMhzToFreqCode(freqVal,NULL)) > (uint16_t)0)
          disp7SegSetOvrAsciiViaWord(wordVal,0);
        else
          disp7SegSetOvrShowDashes(0);    //if no freq code show dashes
      }
#endif
      waitRssiReady();               //delay after channel change
      Serial.print((int)readRssiValue());
      if(++i >= listFreqsMHzArrCount)
        break;
      Serial.print(',');
    }
    Serial.println();
#if DISP7SEG_ENABLED_FLAG
    if(displayConnectedFlag)
      disp7SegClearOvrDisplay();
#endif
    return (uint16_t)0;
  }
  else
  {  //not showing RSSI values for frequencies entered via 'L' command
    Serial.print(' ');
    if(showChanFlag)
    {  //showing channel info
      Serial.print((int)getCurrentFreqInMhz());
      const uint16_t codeVal = getCurrentFreqCodeWord();
      if(codeVal > (uint16_t)0)
      {  //frequency-code value available; show it
        Serial.print((char)(codeVal >> (uint16_t)8));
        Serial.print((char)(codeVal & (uint16_t)0x7F));
      }
      Serial.print('=');
    }
    waitRssiReady();            //make sure not too soon after chan change
    const uint16_t rVal = readRssiValue();
    Serial.print((int)rVal);
    if(monitorModeNextFlag)       //if auto-tune-monitor mode then
      Serial.print(" M");         //append indicator
    Serial.println();
    return rVal;
  }
}

//Repeats the last call to 'showCurrentRssi()' using the same 'showListFlag'
// parameter value.
// Returns:  Last 'showListFlag' parameter value.
boolean repeatShowCurrentRssi()
{
  if(lastShowCurRssiListFlag)
  {  //showing RSSI values for frequencies entered via 'L' command
    if(listFreqsMHzArrCount > 0)
    {  //list not empty
      const uint16_t prevFreqVal = currentTunerFreqMhzOrCode;
      showCurrentRssi(true,false);            //show RSSI for list
      setTunerChannelToFreq(prevFreqVal);     //restore tuner frequency
    }
    return true;
  }
  else
  {  //not showing RSSI values for frequencies entered via 'L' command
    showCurrentRssi(false,false);
    return false;
  }
}

//Process command to change tuned frequency by one MHz.
// upFlag:  true to increment frequency; false to decrement frequency.
// serialOutFlag:  true for serial-output messages.
void processOneMHzCommand(boolean upFlag, boolean serialOutFlag)
{
  uint16_t freqVal = getCurrentFreqInMhz();
  if(upFlag)
  {  //increment frequency
    if(freqVal >= MAX_CHANNEL_MHZ)
    {
      if(serialOutFlag)
        Serial.println(F(" At maximum frequency value"));
      return;
    }
    ++freqVal;
  }
  else
  {  //decrement frequency
    if(freqVal <= MIN_CHANNEL_MHZ)
    {
      if(serialOutFlag)
        Serial.println(F(" At minimum frequency value"));
      return;
    }
    --freqVal;
  }
         //if frequency corresponds to a frequency code then use it:
  uint16_t codeVal;
  if((codeVal=freqInMhzToFreqCode(freqVal,NULL)) != (uint16_t)0)
    setTunerChannelToFreq(codeVal);
  else
    setTunerChannelToFreq(freqVal);
         //save tuned freq to EEPROM 3 secs after last change:
  scheduleDelayedSaveFreqToEeprom(3);

  const uint16_t regVal = freqMhzToRegVal(freqVal);
  if(serialOutFlag)
  {  //serial echo enabled; show extra info
    Serial.print((int)freqVal);
    Serial.print(" [0x");
    Serial.print(itoa((int)regVal,itoaBuff,16));
    Serial.print(']');
    if(codeVal > (uint16_t)0)
    {  //frequency-code value available; show it
      Serial.print(" (");
      Serial.print((char)(codeVal >> (uint16_t)8));
      Serial.print((char)(codeVal & (uint16_t)0x7F));
      Serial.print(')');
    }
    waitRssiReady();            //delay after channel change
    Serial.print("  ");
    Serial.println((int)readRssiValue());
  }
}

//Process command to increment (or decrement) band or channel on
// tuned-frequency code.
// bandFlag:  true for band; false for channel.
// upFlag:  true to increment; false to decrement.
// buttonFlag:  true if input via button; false if via serial command.
void processIncFreqCodeCommand(boolean bandFlag, boolean upFlag,
                                                         boolean buttonFlag)
{
  uint16_t codeVal = getCurrentFreqCodeWord();
  if(codeVal > (uint16_t)0)
  {  //currently tuned at a frequency code word; increment code word
    codeVal = incDecFreqCodeValue(codeVal,bandFlag,upFlag);
  }
  else
  {  //not currently tuned at a frequency code word; move to next one
    codeVal = freqInMhzToNearestFreqCode(getCurrentFreqInMhz(),upFlag,NULL);
  }
  const uint16_t freqInMHz = freqCodeWordToFreqInMhz(codeVal);  //get new MHz
  if(buttonFlag && serialEchoFlag)
  {  //via button and echo on
    Serial.println();             //send newline (to precede tune message)
    setSerialInputPromptFlag();   //setup to show prompt later
  }
         //tune and send status to serial port:
  doTuneToFreqMHzOrCode(freqInMHz,codeVal);
         //save tuned freq to EEPROM 3 secs after last change:
  scheduleDelayedSaveFreqToEeprom(3);
}

//Auto scans frequencies and (successively) tunes to found channels.
// minRssiLevel:  minimum RSSI value for accepted channels.
// scanAndTuneFirstFlag:  if true then a scan is always performed and the
//                        channel with the highest RSSI is tuned.
// scanForwardFlag:  true to scan channels forward; false for backward.
// rescanOnSingleFlag:  if true and only one channel with high enough RSSI
//                      is found then a rescan will be performed on next
//                      iteration.
void doAutoScanTuneChannel(int minRssiLevel, boolean scanAndTuneFirstFlag,
                        boolean scanForwardFlag, boolean rescanOnSingleFlag)
{
  boolean scanDoneFlag = false;
  while(1)
  {  //will loop if tuning channel after scanning frequencies
    if(scanDoneFlag || (!scanAndTuneFirstFlag && idxSortedSelArrCount > 1 &&
              ((minRssiLevel <= 0 && listFreqsMHzArrCount > 0) ||
               millis() < lastNextTuneScanTime + NEXT_CHAN_RESCANSECS*1000)))
    {  //just did freq scan or scanned chans avail and not time to rescan
       // (if 'scanAndTuneFirstFlag'==true then always do scan first)
       // (if 'minRssi'==0 and using 'L' list don't scan because of timeout)
      if(scanForwardFlag)
      {  //scanning forward; increment index (with wrap around)
        if(++nextTuneChannelIndex >= idxSortedSelArrCount)
          nextTuneChannelIndex = 0;    //select next (or first) channel
      }
      else
      {  //scanning backward; decrement index (with wrap around)
        if(nextTuneChannelIndex > 0)
          --nextTuneChannelIndex;
        else if(idxSortedSelArrCount > 0)
          nextTuneChannelIndex = idxSortedSelArrCount - 1;
      }
      if(nextTuneChannelIndex < idxSortedSelArrCount)
      {  //channel index value OK
        const uint8_t chanIdx = idxSortedSelectedArr[nextTuneChannelIndex];
        uint16_t freqVal;
        if(listFreqsMHzArrCount > 0)
        {  //using 'listFreqsMHzArr[]' entered via 'L' command; check index
          freqVal = (chanIdx < listFreqsMHzArrCount) ?
                                     listFreqsMHzArr[chanIdx] : (uint16_t)0;
        }
        else  //not using 'listFreqsMHzArr[]' entered via 'L' command
          freqVal = getChannelFreqTableEntry(chanIdx);  //get freq via index
              //display messages and tune to frequency:
        if(freqVal >= MIN_CHANNEL_MHZ && freqVal <= MAX_CHANNEL_MHZ)
        {  //frequency value is in range
          if(serialEchoFlag)
          {  //serial-echo enabled; show output
            Serial.print(F(" Tuning to frequency "));
            if(!scanAndTuneFirstFlag)
            {  //stepping through channels; show index and total
              Serial.print('(');
              Serial.print(nextTuneChannelIndex+1);
              Serial.print('/');
              Serial.print(idxSortedSelArrCount);
              Serial.print(") ");
            }
            Serial.print(freqVal);
            Serial.print(F("MHz"));
            const uint16_t codeVal = freqInMhzToFreqCode((uint16_t)freqVal,NULL);
            if(codeVal > (uint16_t)0)
            {  //frequency-code value available; show it
              Serial.print(" (");
              Serial.print((char)(codeVal >> (uint16_t)8));
              Serial.print((char)(codeVal & (uint16_t)0x7F));
              Serial.print(')');
            }
          }
          if(freqVal != getCurrentFreqInMhz())
          {  //frequency is different from currently-tuned frequency
                   //if freq corresponds to a frequency code then use it:
            uint16_t codeVal;
            if((codeVal=freqInMhzToFreqCode(freqVal,NULL)) != (uint16_t)0)
              setTunerChannelToFreq(codeVal);
            else
              setTunerChannelToFreq(freqVal);
          }
          else
          {  //frequency same as current; make sure display matches freq
#if DISP7SEG_ENABLED_FLAG
            if(displayConnectedFlag)        //if display wired in then
              showTunerChannelOnDisplay();  //update tuner channel on display
#endif
          }
          waitRssiReady();           //delay after channel change
          const uint8_t rssiVal = readRssiValue();
          if(serialEchoFlag)
          {  //serial-echo enabled; show output
            Serial.print(F(", RSSI="));
            Serial.print((int)rssiVal);
          }
          const boolean rssiGoodFlag = (rssiVal >= minRssiLevel/2);
          if(scanAndTuneFirstFlag)
          {  //scanning and tuning to highest-RSSI channel
            if(!rssiGoodFlag)               //if RSSI low then
              nextTuneChannelIndex = -1;    //rescan on next invocation
            if(serialEchoFlag)
              Serial.println();   //finish display line
            break;                //exit function
          }
          if(scanDoneFlag || rssiGoodFlag)
          {  //scan was just performed or RSSI value is high enough
                        //if flag and only one entry then
                        //setup to rescan on next invocation:
            if(rescanOnSingleFlag && idxSortedSelArrCount <= 1)
              nextTuneChannelIndex = -1;
            if(serialEchoFlag)
              Serial.println();   //finish display line
            if(rssiGoodFlag)
            {  //RSSI value is high enough
              Serial.print('T');   //send tune cmd to possible slave receiver
              Serial.println(freqVal);
            }
            break;                //exit function
          }
              //scan was not just performed and RSSI value is low
          if(serialEchoFlag)                     //do scan of frequencies now
            Serial.println(F(", rescanning"));
        }
        else
        {
          Serial.print(F(" Channel frequency value out of range:  "));
          Serial.println(freqVal);
        }
      }
    }
    if(scanDoneFlag)    //if scan already performed then
      break;            //exit function
    nextTuneChannelIndex = -1;         //setup to select first channel
    const uint16_t prevFreqVal = currentTunerFreqMhzOrCode;
         //if always tuning highest-RSSI channel then use fallback-min
         // value of 0 so all channels are always accepted:
    const int fallbackRssiLevel = scanAndTuneFirstFlag ? 0 : minRssiLevel;
                        //do scan of frequencies:
    if(!scanChannelsAndReport(minRssiLevel,fallbackRssiLevel,false,false,
                                                            serialEchoFlag))
    {  //no channels with high enough RSSI found
      setTunerChannelToFreq(prevFreqVal);   //restore tuner frequency
      break;            //exit function
    }
    scanDoneFlag = true;     //indicate successful scan was performed
  }  //loop around and tune channel
}

//Auto scans frequencies and tunes to highest-RSSI channel.
void processAutoScanAndTuneCommand(const char *valueStr)
{
  if(strlen(valueStr) > 0)
  {  //parameter given; not allowed
    Serial.println(F(" Invalid parameter on Auto-scan command; none allowed"));
    return;
  }
         //do scan (always) and select highest-RSSI channel:
  doAutoScanTuneChannel(sessionDefMinRssiLevel,true,true,true);
         //save tuned freq to EEPROM 3 secs after last set:
  scheduleDelayedSaveFreqToEeprom(3);
}

//Auto scans frequencies and (successively) tunes to found channels.
// valueStr:  Numeric string containing minimum RSSI value for
//            accepted channels, or empty string for default.
// scanForwardFlag:  true to scan channels forward; false for backward.
// rescanOnSingleFlag:  if true and only one channel with high enough RSSI
//                      is found then a rescan will be performed on next
//                      iteration.
void autoScanTuneNextChan(const char *valueStr, boolean scanForwardFlag,
                                                 boolean rescanOnSingleFlag)
{
  int minRssiLevel = sessionDefMinRssiLevel;
  if(strlen(valueStr))
  {  //given value string not empty
    if(!convStrToInt(valueStr,&minRssiLevel))
    {  //error parsing given value
      showUnableToParseValueMsg();
      Serial.println(valueStr);
      return;
    }
    sessionDefMinRssiLevel = minRssiLevel;  //save new default for session
  }
  doAutoScanTuneChannel(minRssiLevel,false,scanForwardFlag,
                                                        rescanOnSingleFlag);
}

//Processes auto-tune-monitor command (scans frequencies and tunes to found
// channels automatically on a periodic basis).
// valueStr:  Numeric string containing cycle time (in seconds),
//            or empty string for default.
void processMonitorModeCommand(const char *valueStr)
{
  int val = monitorModeIntervalSecs;
  if(strlen(valueStr) > 0 && !convStrToInt(valueStr,&val))
  {  //error parsing given value
    showUnableToParseValueMsg();
    Serial.println(valueStr);
    return;
  }
  if(val <= 0)
  {
    Serial.print(F(" Value too low:  "));
    Serial.println(val);
    return;
  }
  monitorModeIntervalSecs = val;
  monitorModeNextFlag = true;
  monitorModeNextChanTime = 0;
  clearSerialInputPromptFlag();        //suppress '>' serial prompt
  monitorAutoTuneNextChan();
}

//Performs the auto-tune-monitor function.  This function is called
// on a periodic basis and handles the timed invocations of the
// 'autoScanTuneNextChan()' function ('N' command).
void monitorAutoTuneNextChan()
{
  const unsigned long curTime = millis();
  uint16_t curRssi = readRssiValue();  //get RSSI of current channel
  long timeOffs = (long)monitorModeIntervalSecs * 1000;
  if(curRssi < MAX_RSSI_VAL*3/10)      //less than 30
  {  //RSSI of current channel is on the low side
    int curIdx;         //get highest RSSI value among all channels:
    if((curIdx=idxSortedByRssiArr[0]) >= CHANNEL_MIN_INDEX &&
                                            curIdx < LISTFREQMHZ_ARR_SIZE &&
                                scanRssiValuesArr[curIdx] >= MAX_RSSI_VAL/2)
    {  //highest RSSI value is on the high side
              //reduce duration that current channel will be shown:
      timeOffs = timeOffs * curRssi / 30;
    }
  }
  if(curTime > monitorModeNextChanTime + timeOffs)
  {  //time reached to switch to next channel
    doAutoScanTuneChannel(sessionDefMinRssiLevel,false,true,false);
    monitorModeNextChanTime = curTime;
    updateActivityIndicator(true);
  }
}

//Scans and reports channels that have RSSI values above the limit.
// If a list of frequencies was entered via the 'L' command then
// it is used (unless the 'inclAllFlag' parameter is true).
// This function blocks while the scanning is performed.
// minRssiLevel:  minimum RSSI value for accepted channels.
// fallbackRssiLevel:  alternate minimum RSSI value to use if RSSI of
//                     all channels is below 'minRssiLevel'.
// inclAllFlag:  true to include all frequencies; false to squelch
//               frequencies adjacent to those already shown.
// restoreFreqFlag:  true to restore tuner frequency on exit.
// showOutputFlag:  show serial-output messages.
//Returns true if one or more channels with high enough RSSI found;
// false if all channels have low RSSI.
boolean scanChannelsAndReport(int minRssiLevel, int fallbackRssiLevel,
       boolean inclAllFlag, boolean restoreFreqFlag, boolean showOutputFlag)
{
         //scan frequencies and store received RSSI values:
  scanChansGetRssiValues(USE_LBAND_FLAG,inclAllFlag,restoreFreqFlag,false);
  loadIdxSortedByRssiArr(inclAllFlag); //create list sorted by RSSI values
  lastNextTuneScanTime = millis();
  int curIdx;           //get highest RSSI value among all channels:
  if((curIdx=idxSortedByRssiArr[0]) >= CHANNEL_MIN_INDEX &&
                                            curIdx < LISTFREQMHZ_ARR_SIZE &&
                                   scanRssiValuesArr[curIdx] < minRssiLevel)
  {  //all RSSI values are below the minimum
    minRssiLevel = fallbackRssiLevel;       //use alternate minimum RSSI
  }
                   //check if should use list entered via 'L' command:
  const boolean listFlag = ((!inclAllFlag) && listFreqsMHzArrCount > 0);
  boolean firstFlag = true;
  int minIdx,maxIdx;
  uint8_t *idxArr;
  if(inclAllFlag)
  {  //show all frequencies (via tables)
    minIdx = CHANNEL_MIN_INDEX;
    maxIdx = CHANNEL_MAX_INDEX;
    idxArr = idxSortedByRssiArr;       //use array with all frequencies
  }
  else if(!listFlag)
  {  //squelch frequencies adjacent to those already shown
    minIdx = CHANNEL_MIN_INDEX;
    maxIdx = loadIdxSortedSelectedArr() - 1;
    idxArr = idxSortedSelectedArr;     //use array with selected indices
  }
  else
  {  //using 'listFreqsMHzArr[]' entered via 'L' command
    minIdx = 0;
    maxIdx = listFreqsMHzArrCount - 1;
    for(int p=0; p<listFreqsMHzArrCount; ++p)    //copy to sel-freqs array
      idxSortedSelectedArr[p] = idxSortedByRssiArr[p];
    idxSortedSelArrCount = listFreqsMHzArrCount;
    idxArr = idxSortedSelectedArr;     //use array with copied indices
  }
  for(int i=minIdx; i<=maxIdx; ++i)
  {  //for each possible channel
    curIdx = idxArr[i];    //get index from sorted list
    if(scanRssiValuesArr[curIdx] >= minRssiLevel)
    {  //RSSI level is high enough
      if(firstFlag)
        firstFlag = false;
      if(showOutputFlag)
      {  //serial-output enabled
        Serial.print(' ');        //put in separator (or leading space)
        if(listFlag)
          Serial.print((int)listFreqsMHzArr[curIdx]);
        else
          Serial.print((int)getChannelFreqTableEntry(curIdx));
        Serial.print('=');
        Serial.print(scanRssiValuesArr[curIdx]);
      }
    }
    else
    {  //RSSI level not high enough
      if(firstFlag)
      {  //none with RSSI high enough
        nextTuneChannelIndex = -1;     //clear any current index
        if(showOutputFlag)
        {  //serial-output enabled
          Serial.print(F(" No channels with RSSI at least "));
          Serial.print(minRssiLevel);
#if DISP7SEG_ENABLED_FLAG              //show indicator on display
          if(displayConnectedFlag)
            disp7SegSetOvrAsciiValues('n',false,'c',false,1000);
#endif
        }
      }
         //reduce size of 'idxSortedSelectedArr[]' array to number
         // of channels above minimum RSSI level:
      if(!inclAllFlag)
        idxSortedSelArrCount = i;
      break;
    }
  }
  if(showOutputFlag)
    Serial.println();
  if(nextTuneChannelIndex > 0 &&      //check RSSI entry for current channel
                              nextTuneChannelIndex < idxSortedSelArrCount &&
             scanRssiValuesArr[idxSortedSelectedArr[nextTuneChannelIndex]] <
                                                               minRssiLevel)
  {  //RSSI of current channel is below minimum; reset to first channel
    nextTuneChannelIndex = -1;     //clear any current index
  }
  flushSerialInputLines();   //clear any commands received while busy scanning
  if(!firstFlag)             //if channel with high enough RSSI found then
    return true;             //return indicator flag
  return false;              //indicate no channels with high enough RSSI
}

//Processes the commands to scan and report channels that have RSSI values
// above the limit.
// valueStr:  Numeric string containing minimum RSSI value to be
//            displayed, or empty string for default.
// inclAllFlag:  true to include all frequencies; false to squelch
//               frequencies adjacent to those already shown.
//Returns true if one or more channels with high enough RSSI found;
// false if all channels have low RSSI.
boolean processScanChannelsCommand(const char *valueStr, boolean inclAllFlag)
{
  int minRssiLevel = sessionDefMinRssiLevel;
  const int sLen = strlen(valueStr);
  int p = 0;
  while(valueStr[p] == ' ' && p < sLen)
    ++p;              //ignore any leading spaces
  if(p < sLen)
  {  //given value string not empty
    if(!convStrToInt(valueStr,&minRssiLevel))
    {  //error parsing given value
      showUnableToParseValueMsg();
      Serial.println(valueStr);
      return false;
    }
    sessionDefMinRssiLevel = minRssiLevel;  //save new default for session
  }
  return scanChannelsAndReport(             //always show output
                           minRssiLevel,minRssiLevel,inclAllFlag,true,true);
}

//Scans channels and stores received RSSI values in the 'scanRssiValuesArr[]'
// array.  If a list of frequencies was entered via the 'L' command then
// it is used (unless the 'inclAllFlag' parameter is true).  This function
// blocks while the scanning is performed.
// includeLBandFlag:  true to include L-band frequencies in scan.
// inclAllFlag:  true to include all frequencies (don't use list entered
//               via 'L' command).
// restoreFreqFlag:  true to restore tuner frequency on exit.
// showOutputFlag:  show messages while scanning.
void scanChansGetRssiValues(boolean includeLBandFlag, boolean inclAllFlag,
                            boolean restoreFreqFlag, boolean showOutputFlag)
{
  uint16_t prevFreqVal = 0;
  if(restoreFreqFlag)
    prevFreqVal = currentTunerFreqMhzOrCode;
  clearRssiOutput();         //clear analog-RSSI output
                   //check if should use list entered via 'L' command:
  const boolean listFlag = ((!inclAllFlag) && listFreqsMHzArrCount > 0);
  Serial.print(" Scanning");
  int idx, maxIdx;
  if(listFlag)
  {  //using 'listFreqsMHzArr[]' entered via 'L' command
    idx = 0;
    maxIdx = listFreqsMHzArrCount - 1;
  }
  else
  {  //not using 'listFreqsMHzArr[]' entered via 'L' command
    idx = CHANNEL_MIN_INDEX;      //using full list of channels from table
    maxIdx = CHANNEL_MAX_INDEX;
  }
  int tableIdx = 0;
  uint16_t freqVal,wordVal;
  while(true)
  {  //for each channel slot
    if(listFlag)
    {  //using 'listFreqsMHzArr[]' entered via 'L' command
      freqVal = listFreqsMHzArr[idx];
      tableIdx = idx;        //index into 'scanRssiValuesArr[]' array
    }
    else
    {  //not using 'listFreqsMHzArr[]' entered via 'L' command
      tableIdx = (int)getChannelSortTableEntry(idx);    //sort by MHz
         //if including L-band or not L-band channel then get freq in MHz:
      freqVal = (includeLBandFlag || !isLBandChannelIndex(tableIdx)) ?
                           getChannelFreqTableEntry(tableIdx) : (uint16_t)0;
    }
    if(freqVal >= MIN_CHANNEL_MHZ && freqVal <= MAX_CHANNEL_MHZ)
    {  //frequency is valid
      setCurrentFreqByMhzOrCode(freqVal);
#if DISP7SEG_ENABLED_FLAG    //display frequency codes while scanning
      if(displayConnectedFlag)
      {
        if(listFlag)
        {  //using 'listFreqsMHzArr[]' entered via 'L' command
                   //display freq code for freq value (if any):
          if((wordVal=freqInMhzToFreqCode(freqVal,NULL)) > (uint16_t)0)
            disp7SegSetOvrAsciiViaWord(wordVal,0);
          else
            disp7SegSetOvrShowDashes(0);    //if no freq code show dashes
        }
        else
        {  //not using 'listFreqsMHzArr[]' entered via 'L' command
          disp7SegSetOvrAsciiViaWord(freqIdxToFreqCode(tableIdx,NULL), 0);
        }
      }
#endif
      waitRssiReady();               //delay after channel change
      scanRssiValuesArr[tableIdx] = (uint8_t)readRssiValue();
      if(showOutputFlag)
      {
        Serial.print((int)freqVal);
        Serial.print('(');
        Serial.print(tableIdx);
        Serial.print(")=");
        Serial.print((int)scanRssiValuesArr[tableIdx]);
      }
      if(++idx > maxIdx)
        break;
      if(showOutputFlag)
        Serial.print(',');
      else if((idx % 8) == 0)
        Serial.print(".");
    }
    else
    {  //frequency value not valid (skipping L-band channel)
      scanRssiValuesArr[tableIdx] = (uint8_t)0;
      if(++idx > maxIdx)
        break;
    }
    if(!displayConnectedFlag)               //if no display then
      updateActivityIndicator(true);        //indicate "extra" activity
  }
  Serial.println();
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
    disp7SegClearOvrDisplay();    //clear displayed freq code
#endif
  if(restoreFreqFlag)
    setTunerChannelToFreq(prevFreqVal);
#if BUTTONS_ENABLED_FLAG
  fetchButtonsTriggerState();     //clear any presses while busy scanning
#endif
}

//Tunes to the given channel, receives its RSSI value, and displays it.
// freqVal:  frequency value to scan.
// tableIdx:  table index for frequency, or -1 if none.
void doScanFreqShowRssiValue(uint16_t freqVal, int tableIdx)
{
  uint16_t rssiVal;
  setCurrentFreqByMhzOrCode(freqVal);
  uint16_t codeVal = (tableIdx >= 0) ?      //get code for freq (if avail)
                             freqIdxToFreqCode(tableIdx,NULL) : (uint16_t)0;
#if DISP7SEG_ENABLED_FLAG    //display frequency codes while scanning
  if(displayConnectedFlag)
  {
    if(codeVal > (uint16_t)0)          //show freq code (via table index):
      disp7SegSetOvrAsciiViaWord(codeVal,0);
    else
      disp7SegSetOvrShowDashes(0);     //if no freq code show dashes
  }
#endif
  waitRssiReady();                   //delay after channel change
  rssiVal = (uint8_t)readRssiValue();
  Serial.print((int)freqVal);
  if(codeVal > (uint16_t)0)
  {  //frequency-code value available; show it
    Serial.print((char)(codeVal >> (uint16_t)8));
    Serial.print((char)(codeVal & (uint16_t)0x7F));
  }
  Serial.print('=');
  Serial.println((int)rssiVal);
  if(!displayConnectedFlag)            //if no display then
    updateActivityIndicator(true);     //indicate "extra" activity
}

//Performs a full-band scan and displays received RSSI values while scanning.
// This function blocks while the scanning is performed.
void fullScanShowRssiValues()
{
  clearRssiOutput();         //clear analog-RSSI output
  uint16_t nextFreqVal, freqVal = 0;
  int diff;
  int idx = CHANNEL_MIN_INDEX - 1;
  int tableIdx = -1;
  nextFreqVal = getChannelFreqTableEntry(   //start at minFreq-37 MHz
                          getChannelSortTableEntry(CHANNEL_MIN_INDEX)) - 37;
  while(true)
  {  //for each entry in table of channels sorted by MHz value
    if(nextFreqVal != freqVal)    //if not same as last freq then do scan
      doScanFreqShowRssiValue(nextFreqVal,tableIdx);
    freqVal = nextFreqVal;
    if(++idx <= CHANNEL_MAX_INDEX)
    {  //next index is in range
      tableIdx = (int)getChannelSortTableEntry(idx);       //channel index
      nextFreqVal = getChannelFreqTableEntry(tableIdx);    //freq in MHz
    }
    else
    {  //all frequencies in table have been processed
      if(idx > CHANNEL_MAX_INDEX+1)    //if additional freq has been processed
        break;                         // then exit loop
      tableIdx = -1;
      nextFreqVal += 37;     //finish at maxFreq+37 MHz
    }
         //if spacing between current and next frequency is
         // wide enough then add fill-in frequency in between:
    if((diff=nextFreqVal-freqVal) > 35)
    {  //if 37 MHz between L-band freqs put 3 in between
      diff /= 4;
      freqVal += diff;
      doScanFreqShowRssiValue(freqVal,-1);
      freqVal += diff+1;
      doScanFreqShowRssiValue(freqVal,-1);
      freqVal += diff;
      doScanFreqShowRssiValue(freqVal,-1);
    }
    else if(diff > 22)
    {  //enough room for two frequencies in between
      diff /= 3;
      freqVal += diff;
      doScanFreqShowRssiValue(freqVal,-1);
      freqVal += diff;
      doScanFreqShowRssiValue(freqVal,-1);
    }
    else if(diff > 9)
    {  //enough room for one frequency in between
      freqVal += diff / 2;        //do average of values
      doScanFreqShowRssiValue(freqVal,-1);
    }
    if(Serial.available())        //if any serial input then
      break;                      //abort scan
  }
  Serial.println("0=0");          //show "finished" indicator
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
    disp7SegClearOvrDisplay();    //clear displayed freq code
#endif
    //do tune to leave display, etc correctly in sync
         //see if frequency corresponds to a frequency code:
  const uint16_t codeVal = freqInMhzToFreqCode(freqVal,NULL);
         //if matching code then use it; otherwise freq value:
  setTunerChannelToFreq((codeVal != (uint16_t)0) ? codeVal : freqVal);
#if BUTTONS_ENABLED_FLAG
  fetchButtonsTriggerState();     //clear any presses while busy scanning
#endif
}

//Process command for serial echo on/off or echo text.
void processSerialEchoCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen > 0)
  {  //parameter value given
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //skip leading spaces
    if(p < sLen)
    {  //given parameter not empty
      const char ch = valueStr[p];
      if(ch == '0')
        serialEchoFlag = false;        //turn off echo
      else if(ch == '1')
        serialEchoFlag = true;         //turn on echo
      else
        Serial.println(&valueStr[p]);  //echo given text
      return;
    }
  }
    //no parameter; show current echo on/off value
  Serial.print(' ');
  Serial.println(serialEchoFlag ? 1 : 0);
}

//Processes command to set or show raw-RSSI-scaling values.
void processRawRssiMinMaxCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen > 0)
  {  //parameter value given; set new values
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //ignore any leading spaces
    int minVal = (int)getRx5808RawRssiMinVal();  //use current if not specified
    int maxVal = (int)getRx5808RawRssiMaxVal();
    if(p < sLen)
    {  //given parameter-string not empty
      if(strncasecmp(&valueStr[p],"default",7) != 0)
      {  //given parameter-string is not "default"
        int q = p;
              //scan until space, comma or end of string:
        while(q < sLen && valueStr[q] != ' ' && valueStr[q] != ',')
          ++q;
              //scan through any extra spaces before comma:
        while(valueStr[q] == ' ' && ++q < sLen);
              //if comma found then scan through any spaces after it:
        if(q < sLen && valueStr[q] == ',')
          while(++q < sLen && valueStr[q] == ' ');
              //parse first value at 'p' and second at 'q' (if given):
        if((valueStr[p] == ',' ||
                     (convStrToInt(&valueStr[p],&minVal) && minVal >= 0)) &&
                                                               (q >= sLen ||
                       (convStrToInt(&valueStr[q],&maxVal) && maxVal >= 0)))
        {  //values not given or parsed OK
          if(minVal >= maxVal)
          {  //min is not less than max
            Serial.println(F(" Min value cannot be less than max value"));
            return;          //don't show values below
          }
        }
        else
        {
          showUnableToParseValueMsg();
          Serial.println(valueStr);
          return;            //don't show values below
        }
      }
      else
      {  //given parameter-string is "default"
        minVal = DEF_RAWRSSI_MIN;      //use default values
        maxVal = DEF_RAWRSSI_MAX;
      }
      if(minVal != getRx5808RawRssiMinVal() ||
                                         maxVal != getRx5808RawRssiMaxVal())
      {  //new values different from current ones; accept values
        setRx5808RawRssiMinMax((uint16_t)minVal,(uint16_t)maxVal);
        saveRssiMinMaxValsToEeprom();            //save values
      }
      if(!serialEchoFlag)      //if serial echo disabled then
        return;                //don't show values below
    }
  }
    //show current values:
  Serial.print(' ');
  if(serialEchoFlag)
    Serial.print(F("RSSI-scaling values:  "));
  Serial.print((int)getRx5808RawRssiMinVal());
  Serial.print(',');
  Serial.println((int)getRx5808RawRssiMaxVal());
}

//Processes command to disable/enable/restart auto RSSI calibration.
void processEnableAutoRssiCalibCmd(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  boolean restartFlag = false;
  if(sLen > 0)
  {  //parameter value given; set new value
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //skip leading spaces
    if(p < sLen)
    {  //given parameter not empty
      const char ch = valueStr[p];
      boolean newFlag;
      if(ch == '0')
        newFlag = false;
      else if(ch == '1')
        newFlag = true;
      else if(ch == 'R' || ch == 'r')
      {  //restart calibration; set min/max to default values
        if(getRx5808RawRssiMinVal() != DEF_RAWRSSI_MIN ||
                                getRx5808RawRssiMaxVal() != DEF_RAWRSSI_MAX)
        {  //currently not at default values; enter them
          setRx5808RawRssiMinMax(DEF_RAWRSSI_MIN,DEF_RAWRSSI_MAX);
          saveRssiMinMaxValsToEeprom();          //save values
        }
        restartFlag = true;
        newFlag = true;      //always be enabled after restart
        autoRssiCalibShowOutputFlag = true;      //show data on serial out
      }
      else if(ch == 'S' || ch == 's')
      {  //show calibration changes
        autoRssiCalibShowOutputFlag = true;      //show data on serial out
        newFlag = autoRssiCalibEnabledFlag;      //leave state unchanged
      }
      else
      {
        Serial.println(F(" Invalid value (must be 0, 1 or R)"));
        return;
      }
      if(newFlag != autoRssiCalibEnabledFlag)
      {  //value is changing; save new value
        autoRssiCalibEnabledFlag = newFlag;
        saveAutoRssiCalFlagToEeprom(newFlag);
      }
      if(!serialEchoFlag)
        return;         //if serial-echo off then don't show output
    }
  }
    //show current value:
  Serial.print(' ');
  if(serialEchoFlag)
  {
    Serial.print(F("Auto RSSI calibration "));
    if(restartFlag)
      Serial.print(F("restarted and "));
    if(autoRssiCalibEnabledFlag)
      Serial.println(F("enabled"));
    else
      Serial.println(F("disabled"));
  }
  else
    Serial.println(autoRssiCalibEnabledFlag ? 1 : 0);
}

//Processes command to set or show RX5808 minimum-tune time (ms).
// valueStr:  Numeric string containing RX5808 minimum-tune time (ms),
//            empty string to show current value, or "default" for
//            default value.
void processMinTuneTimeCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  int p = 0;
  while(valueStr[p] == ' ' && p < sLen)
    ++p;              //ignore any leading spaces
  if(p < sLen)
  {  //given value string not empty
    int val = RX5808_MIN_TUNETIME;
    const boolean defFlag = (strncasecmp(&valueStr[p],"default",7) == 0);
    if(!defFlag)
    {  //given parameter-string is not "default"
      if(!convStrToInt(valueStr,&val))
      {  //error parsing given value
        showUnableToParseValueMsg();
        Serial.println(valueStr);
        return;
      }
      if(val < 0 || val >= 255)
      {  //value out of range
        Serial.print(F(" Entered value out of range:  "));
        Serial.println(valueStr);
        return;
      }
    }
    setRx5808MinTuneTimeMs((uint8_t)val);        //set value
    saveMinTuneTimeMsToEeprom((uint8_t)val);     //save to EEPROM
    if(!defFlag)
      return;
  }
    //no parameter (or "default") given
  Serial.print(' ');
  if(serialEchoFlag)
    Serial.print(F("RX5808 minimum-tune time (ms):  "));
  Serial.println((int)getRx5808MinTuneTimeMs());
}

//Processes command to set or show minimum-RSSI value for scans.
// valueStr:  Numeric string containing minimum RSSI value to be
//            displayed, or empty string to show current value.
void processMinRssiCommand(const char *valueStr)
{
  int minRssiLevel = sessionDefMinRssiLevel;
  const int sLen = strlen(valueStr);
  int p = 0;
  while(valueStr[p] == ' ' && p < sLen)
    ++p;              //ignore any leading spaces
  if(p < sLen)
  {  //given value string not empty
    if(!convStrToInt(valueStr,&minRssiLevel))
    {  //error parsing given value
      showUnableToParseValueMsg();
      Serial.println(valueStr);
      return;
    }
    sessionDefMinRssiLevel = minRssiLevel;  //save new default for session
  }
  else
  {  //no parameter given
    Serial.print(' ');
    if(serialEchoFlag)
      Serial.print(F("Minimum RSSI value:  "));
    Serial.println(sessionDefMinRssiLevel);
  }
}

//Processes command to set or show monitor-mode interval (in seconds).
// valueStr:  Numeric string containing monitor-mode interval (in
//            seconds), or empty string to show current value.
void processMonitorIntervalCmd(const char *valueStr)
{
  int val = monitorModeIntervalSecs;
  const int sLen = strlen(valueStr);
  int p = 0;
  while(valueStr[p] == ' ' && p < sLen)
    ++p;              //ignore any leading spaces
  if(p < sLen)
  {  //given value string not empty
    if(!convStrToInt(valueStr,&val))
    {  //error parsing given value
      showUnableToParseValueMsg();
      Serial.println(valueStr);
      return;
    }
    monitorModeIntervalSecs = val;     //save new value for session
  }
  else
  {  //no parameter given
    Serial.print(' ');
    if(serialEchoFlag)
      Serial.print(F("Monitor-mode interval (seconds):  "));
    Serial.println(monitorModeIntervalSecs);
  }
}

//Sets or shows the Unit-ID string.
void processUnitIdCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen > 0)
  {  //parameter value given; set new Unit-ID string
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //ignore any leading spaces
    saveUnitIdToEeprom(&valueStr[p]);
    if(!serialEchoFlag)      //if serial echo disabled then
      return;                //don't show Unit=ID string below
  }
    //show current Unit-ID string:
  Serial.print(' ');
  if(serialEchoFlag)
    Serial.print(F("Unit ID: "));
  showUnitIdFromEeprom();
  Serial.println();
}

//Performs a soft program restart (and possibly setting config to defaults).
void processSoftRebootCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen > 0)
  {  //parameter value given; set new values
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //ignore any leading spaces
    if(p < sLen)
    {  //given parameter-string not empty
      if(strncasecmp(&valueStr[p],"defaults",8) != 0)
      {  //given parameter-string is not "defaults"
        Serial.print(F(" Unrecognized parameter:  "));
        Serial.println(valueStr);
        return;
      }
      setEepromToDefaultsValues();
    }
  }
  doShutdownCleanup();       //disconnect interrupts
  delay(5);                  //make sure interrupts have cleared
  doSoftwareReset();         //do soft restart
}

//Processes command to show frequency list for preset name.
void processShowFreqPresetListCmd(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  int sPos = 0;
  while(valueStr[sPos] == ' ' && ++sPos < sLen);
  if(sPos >= sLen)
  {  //no parameters given
    return;
  }
  freqListPresetShowForName(&valueStr[sPos]);
}

//Processes command to translate given list of frequencies to
// frequency-index values.  (Developer-helper function.)
void processListTranslateInfoCmd(const char *listStr)
{
  const int sLen = strlen(listStr);
  int sPos = 0;
  while(listStr[sPos] == ' ' && ++sPos < sLen);  //ignore any leading spaces
  if(sPos >= sLen)
  {  //no parameters given
    return;
  }
  int numItems = 0;

  int val, ePos;
  char ch;
  numItems = listFreqsMHzArrCount = 0;
  const int listStrLen = strlen(listStr);
  while(true)
  {  //for each item in list
    while(sPos < listStrLen && ((ch=listStr[sPos]) < '0' || ch > '9'))
    {  //scan through any non-digit chars before digits
      ++sPos;
    }
    if(sPos >= listStrLen)           //if end of input string then
      break;                         //exit loop
    ePos = sPos;                //scan through digits
    while(ePos < listStrLen && (ch=listStr[++ePos]) >= '0' && ch <= '9');
    if(!convStrToInt(&listStr[sPos],&val))
    {  //error parsing as numeric
      Serial.print(F(" Error processing value(s):  "));
      Serial.println(&listStr[sPos]);
      break;
    }
    if(val < MIN_CHANNEL_MHZ || val > MAX_CHANNEL_MHZ)
    {  //value out of range
      if(val != 0 || numItems > 0)
      {  //not leading zero value (for list clear)
        Serial.print(F(" Entered value out of range:  "));
        Serial.println(val);
      }
      break;
    }
    if(numItems >= LISTFREQMHZ_ARR_SIZE)
    {
      Serial.println(F(" Too many values specified"));
      break;
    }
    listFreqsMHzArr[numItems++] = (uint16_t)val;
    sPos = ePos;
  }
  if(numItems > 0)
  {
    sPos = 0;
    while(true)
    {
      Serial.print(' ');
      Serial.print(getIdxForFreqInMhz(listFreqsMHzArr[sPos]));
      if(++sPos >= numItems)
        break;
      Serial.print(',');
    }
    Serial.println();
  }
  loadListFreqsMHzArrFromEeprom();     //restore list via EEPROM copy
}

//Loads the 'idxSortedByRssiArr[]' array with a list of channel-index
// values, sorted by the RSSI values in 'scanRssiValuesArr[]'.
void loadIdxSortedByRssiArr(boolean inclAllFlag)
{
  idxSortedSelArrCount = 0;  //idxSortedSelectedArr[] values no longer valid
  int minIdx,maxIdx;
  if((!inclAllFlag) && listFreqsMHzArrCount > 0)
  {  //using 'listFreqsMHzArr[]' entered via 'L' command
    minIdx = 0;
    maxIdx = listFreqsMHzArrCount - 1;
  }
  else
  {  //not using 'listFreqsMHzArr[]' entered via 'L' command
    minIdx = CHANNEL_MIN_INDEX;      //using full list of channels
    maxIdx = CHANNEL_MAX_INDEX;
  }
  int idx, sortedArrIdx = 0;
  uint8_t curMaxRssi, lastMaxRssi = 255;
  for(int cnt=minIdx; cnt<=maxIdx; ++cnt)
  {  //loop while filling 'idxSortedByRssiArr[]' (max 'cnt' for safety)
    curMaxRssi = 0;
    idx = minIdx;
    do
    {  //find current maximum
      if(scanRssiValuesArr[idx] > curMaxRssi &&
                                       scanRssiValuesArr[idx] < lastMaxRssi)
      {  //found new maximum that is not larger then last maximum
        curMaxRssi = scanRssiValuesArr[idx];
      }
    }
    while(++idx <= maxIdx);
    idx = minIdx;
    do
    {  //find values matching current maximum
      if(scanRssiValuesArr[idx] == curMaxRssi)
      {  //value matches current maximum; add index to array
        idxSortedByRssiArr[sortedArrIdx++] = (uint8_t)idx;
//        Serial.print(getChannelFreqTableEntry(idx));
//        Serial.print('(');
//        Serial.print(idx);
//        Serial.print(")=");
//        Serial.print(curMaxRssi);
//        Serial.print(' ');
      }
    }
    while(++idx <= maxIdx);
    if(sortedArrIdx > maxIdx || curMaxRssi == 0)
      break;       //exit if array filled or current max is zero
    lastMaxRssi = curMaxRssi;     //save max for next iteration
  }
//  Serial.println();
}

//Loads the 'idxSortedSelectedArr[]' array with a list of channel-index
// values selected from the 'idxSortedByRssiArr[]' array by squelching
// frequencies adjacent to those already loaded.  The channels are
// sorted by the RSSI values in 'scanRssiValuesArr[]'.
int loadIdxSortedSelectedArr()
{
  int selIdx = 0;
  uint8_t curIdx;
  for(int idx=CHANNEL_MIN_INDEX; idx<=CHANNEL_MAX_INDEX; ++idx)
  {  //for each channel-index value in 'idxSortedByRssiArr[]' array
    curIdx = idxSortedByRssiArr[idx];
    int j = 0;
    while(true)
    {  //loop while comparing channel-freq value to values already loaded
      if(j >= selIdx)
      {  //no more loaded values left to check (or is first one)
        idxSortedSelectedArr[selIdx++] = curIdx;      //load channel index
        break;
      }
      if(abs((int)getChannelFreqTableEntry(curIdx)-
                (int)getChannelFreqTableEntry(idxSortedSelectedArr[j])) <=
                                                             ADJ_CHAN_MHZ)
      {  //channel-frequency value is too close to an already-load freq
        break;     //discard channel
      }
      ++j;    //increment to next entry in 'idxSortedByRssiArr[]' array
    }
  }
         //save and return # of entries in 'idxSortedByRssiArr[]' array:
  idxSortedSelArrCount = selIdx;
  return selIdx;
}

//Processes command to show raw input values.
void processShowInputsCmd(const char *valueStr)
{
//  const int sLen = strlen(valueStr);
//  if(sLen > 0)
//  {  //parameter(s) given
//    int p = 0;
//    while(valueStr[p] == ' ' && p < sLen)
//      ++p;              //ignore any leading spaces
//    int numItems = 0;
//    if(valueStr[p] == 'I' || valueStr[p] == 'i')
//    {  //set flag for continuous display
//    }
//  }
  showDebugInputs();
}

//Shows raw input values.
void showDebugInputs()
{
    //D2 <- CH/Up, D3 <- FR/Down, A7 <- RSSI
  Serial.print(" D2=");
  Serial.print(digitalRead(2));
  Serial.print(", D3=");
  Serial.print(digitalRead(3));
  Serial.print(", D4=");
  Serial.print(digitalRead(4));
  analogRead(A5);                 //do pre-reads to help settle inputs
  Serial.print(", A5=");
  Serial.print(analogRead(A5));
  analogRead(A6);
  Serial.print(", A6=");
  Serial.print(analogRead(A6));
  analogRead(A7);
  Serial.print(", A7=");
  Serial.println(analogRead(A7));
}

#if DISP7SEG_ENABLED_FLAG
//Processes debug command to write raw values to displays.
void processWriteDisplayCmd(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen > 0)
  {  //parameter(s) given
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //ignore any leading spaces
    if(p < sLen)
    {
      const char leftCh = valueStr[p];
      const char rightCh = (++p < sLen) ? valueStr[p] : ' ';
      disp7SegSetOvrAsciiValues(leftCh,false,rightCh,false,0);
      return;
    }
  }
  disp7SegSetOvrAsciiValues(' ',false,' ',false,1);
}
#endif

//Steps through table values comparing register values via
// 'freqMhzToRegVal()' vs table and sends report to serial port.
void checkReportTableValues()
{
  int tableIdx;
  uint16_t freqVal, calcRegVal, tableRegVal;

  for(int idx=CHANNEL_MIN_INDEX; idx<=CHANNEL_MAX_INDEX; ++idx)
  {
//    tableIdx = (int)getChannelSortTableEntry(idx);    //sort by MHz
    tableIdx = idx;
    freqVal = getChannelFreqTableEntry(tableIdx);
    calcRegVal = freqMhzToRegVal(freqVal);
    tableRegVal = getChannelRegTableEntry(tableIdx);
    Serial.print(' ');
    Serial.print(freqVal);
    Serial.print(F("MHz : calc=0x"));
    Serial.print(itoa(calcRegVal,itoaBuff,16));
    Serial.print(F(" table=0x"));
    Serial.print(itoa(tableRegVal,itoaBuff,16));
    Serial.print(F(" (="));
    Serial.print(regValToFreqMhz(calcRegVal));
    Serial.print(')');
    if(calcRegVal == tableRegVal)
      Serial.println(F("  OK"));
    else
    {
      if(abs(calcRegVal-tableRegVal) == 1)
        Serial.println(F("  Off by 1MHz"));
      else
        Serial.println(F("  Mismatch"));
    }
  }
}

//Sets tuner frequency to given frequency or frequency code word.
// freqMhzOrCode:  Frequency value in MHz, or two-character frequency
//                 code packed into 2-byte word (high byte is band
//                 character).
void setCurrentFreqByMhzOrCode(uint16_t freqMhzOrCode)
{
  uint16_t freqInMHz;
  if(freqMhzOrCode >= FREQ_CODEWORD_CHECKVAL)
  {  //given value looks like code word; convert to freq in MHz
    if((freqInMHz=freqCodeWordToFreqInMhz(freqMhzOrCode)) == 0)
      freqInMHz = freqMhzOrCode;       //if can't conv use as freq value
  }
  else
    freqInMHz = freqMhzOrCode;         //given value is freq in MHz
  setChannelByFreq(freqInMHz);                   //set tuner
  currentTunerFreqMhzOrCode = freqMhzOrCode;     //save freq MHz or code
  currentTunerFreqInMhz = freqInMHz;             //save freq in MHz
}

//Returns the current tuner frequency, in MHz.
uint16_t getCurrentFreqInMhz()
{
  if(currentTunerFreqInMhz > 0)        //if contains valid value then
    return currentTunerFreqInMhz;      //return freq-in-MHz value
         //fallback if saved MHz value not valid (shouldn't happen):
  if(currentTunerFreqMhzOrCode < FREQ_CODEWORD_CHECKVAL)
    return currentTunerFreqMhzOrCode;  //if not code word then return MHz
  uint16_t freqInMHz;        //convert code word to freq in MHz:
  if((freqInMHz=freqCodeWordToFreqInMhz(currentTunerFreqMhzOrCode)) == 0)
    freqInMHz = currentTunerFreqMhzOrCode;  //if can't conv use as freq value
  return freqInMHz;
}

//Returns the current tuner frequency code-word value, or 0 if none
// available.
uint16_t getCurrentFreqCodeWord()
{
  return (currentTunerFreqMhzOrCode >= FREQ_CODEWORD_CHECKVAL) ?
                                    currentTunerFreqMhzOrCode : (uint16_t)0;
}

//Sets tuner frequency and updates display.
// freqMhzOrCode:  Frequency value in MHz, or two-character frequency
//                 code packed into 2-byte word (high byte is band
//                 character).
void setTunerChannelToFreq(uint16_t freqMhzOrCode)
{
  setCurrentFreqByMhzOrCode(freqMhzOrCode);

#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)             //if display wired in then
    showTunerChannelOnDisplay();       //show tuner channel on display
#endif
}

//Updates the analog-RSSI output and (possibly) the display
// based on the RSSI input (averaged).
// This function should be called on a periodic basis.
void updateRssiOutput()
{
  rssiOutSamplingAvgrTotal += sampleRawRssiValue();
  if(++rssiOutSamplingAvgrCounter >= RSSI_SAMPAVG_COUNT)
  {  //enough samples received; send average to output
    uint16_t rawAvgVal =
            (uint16_t)(rssiOutSamplingAvgrTotal/rssiOutSamplingAvgrCounter);
    updateRssiOutValue(scaleRawRssiValue(rawAvgVal));
    if(autoRssiCalibEnabledFlag)            //if auto-calib enabled then
      processAutoRssiCalValue(rawAvgVal);   //process received value
    rssiOutSamplingAvgrTotal = 0;
    rssiOutSamplingAvgrCounter = 0;
  }
}

//Clears the analog-RSSI output and averager.
void clearRssiOutput()
{
  rssiOutSamplingAvgrTotal = 0;
  rssiOutSamplingAvgrCounter = 0;
  updateRssiOutValue((uint16_t)0);
}

#if BUTTONS_ENABLED_FLAG

//Processes command to set/show button mode.
void processButtonModeCommand(const char *valueStr)
{
  const int sLen = strlen(valueStr);
  if(sLen > 0)
  {  //parameter(s) given
    int p = 0;
    while(valueStr[p] == ' ' && p < sLen)
      ++p;              //ignore any leading spaces
    if(p < sLen)
    {  //non-space parameter(s) given
      const byte newModeVal = (byte)(valueStr[p] - '0');
      if(newModeVal < BTNFN_MODE_MINVAL || newModeVal > BTNFN_MODE_MAXVAL)
      {
        Serial.print(F(" Button-mode value out of range:  "));
        Serial.println(&valueStr[p]);
        return;
      }
      if(newModeVal != buttonsFunctionModeValue)
      {  //given mode value different from current
        if(serialEchoFlag)
        {  //show output if echo enabled
          Serial.print(F(" Setting button mode to "));
          Serial.println((int)newModeVal);
        }
        buttonsFunctionModeValue = newModeVal;
#if DISP7SEG_ENABLED_FLAG         //show new mode value on display
        if(displayConnectedFlag)
          showButtonModeOnDisplay(buttonsFunctionModeValue,1000);
#endif
        saveButtonModeToEeprom(buttonsFunctionModeValue);
        return;
      }
    }
  }
  Serial.print(' ');
  if(serialEchoFlag)
    Serial.print(F("Current button mode is "));
  Serial.println((int)buttonsFunctionModeValue);
#if DISP7SEG_ENABLED_FLAG         //show mode value on display
  if(displayConnectedFlag)
    showButtonModeOnDisplay(buttonsFunctionModeValue,1000);
#endif
}

//Fetches current LOW/HIGH state of buttons.
// Returns:  One of:  NO_BUTTONS_MASK, UP_BUTTON_MASK, DOWN_BUTTON_MASK,
//           BOTH_BUTTONS_MASK.
byte fetchButtonsCurrentState()
{
#if BUTTONPINS_UP2DOWN3_FLAG
  return ((getD2InputCurrentState() == LOW) ? UP_BUTTON_MASK : (byte)0) |
           ((getD3InputCurrentState() == LOW) ? DOWN_BUTTON_MASK : (byte)0);
#elif BUTTONPINS_UP3DOWN2_FLAG
  return ((getD3InputCurrentState() == LOW) ? UP_BUTTON_MASK : (byte)0) |
           ((getD2InputCurrentState() == LOW) ? DOWN_BUTTON_MASK : (byte)0);
#else         //if not using D2/D3 then do polling:
  return ((digitalRead(UP_BUTTON_PIN) == LOW) ? UP_BUTTON_MASK : (byte)0) |
       ((digitalRead(DOWN_BUTTON_PIN) == LOW) ? DOWN_BUTTON_MASK : (byte)0);
#endif
}

//Fetches current trigger states of buttons.  A mask bit will be returned
// if a high-to-low transition on a corresponding button has been detected
// since the last time this function was called.
// Returns:  One of:  NO_BUTTONS_MASK, UP_BUTTON_MASK, DOWN_BUTTON_MASK,
//           BOTH_BUTTONS_MASK.
byte fetchButtonsTriggerState()
{
#if BUTTONPINS_UP2DOWN3_FLAG
  return (getD2InputTriggeredFlag() ? UP_BUTTON_MASK : (byte)0) |
                   (getD3InputTriggeredFlag() ? DOWN_BUTTON_MASK : (byte)0);
#elif BUTTONPINS_UP3DOWN2_FLAG
  return (getD3InputTriggeredFlag() ? UP_BUTTON_MASK : (byte)0) |
                   (getD2InputTriggeredFlag() ? DOWN_BUTTON_MASK : (byte)0);
#else
  return NO_BUTTONS_MASK;    //if not using D2/D3 then no triggering
#endif
}

//Reads state of buttons and returns bitmask values.
// Function should be called on a periodic basis.
// clearButtonClickOnLongPressFlag:  if false then after a long press of
//     a single button is released a button-mask value will be returned.
// Returns:  One of:  NO_BUTTONS_MASK, UP_BUTTON_MASK, DOWN_BUTTON_MASK,
//           BOTH_BUTTONS_MASK, along with possible LONGPRESS_MASK bits.
byte readButtonsState(boolean clearButtonClickOnLongPressFlag)
{
  static byte buttonsInputCurrentMask = 0;       //"live" state of buttons
  static byte buttonsInputDetectedMask = 0;      //detected button-presses mask
  static byte buttonsInputTrackedMask = 0;       //tracked state for debounce
  static byte buttonsInputLongPressMask = 0;     //track held long-presses
  static unsigned long buttonsInputLastChangeTime = millis();

  const unsigned long curTimeMs = millis();
  const byte curMask = fetchButtonsCurrentState();     //get current state
  byte trigMask;             //check if any triggers since last time
  if((trigMask=fetchButtonsTriggerState()) != (byte)0 &&
        (buttonsInputTrackedMask & BOTH_BUTTONS_MASK & trigMask) != (byte)0)
  {  //trigger detected while button tracked as active; ignore trigger
    trigMask &= ~(buttonsInputTrackedMask & BOTH_BUTTONS_MASK & trigMask);
  }
  if(curMask != buttonsInputCurrentMask || trigMask != (byte)0)
  {  //buttons changed since last time through or trigger detected
    buttonsInputLastChangeTime = curTimeMs; //reset last-change time
         //if not using interrupts then detect triggers via polling:
#if !BUTTONPINS_USEINTERRUPT_FLAG
    if((curMask & BOTH_BUTTONS_MASK & ~buttonsInputCurrentMask) != (byte)0)
      trigMask = curMask & BOTH_BUTTONS_MASK & ~buttonsInputCurrentMask;
#endif
    buttonsInputCurrentMask = curMask;      //update current-state mask
    buttonsInputDetectedMask |= trigMask;   //keep track of presses
    return buttonsInputLongPressMask;       //no change to long-presses
  }
  if((buttonsInputCurrentMask == buttonsInputTrackedMask &&
                             buttonsInputDetectedMask == NO_BUTTONS_MASK) ||
            curTimeMs - buttonsInputLastChangeTime < BUTTON_DEBOUNCE_TIMEMS)
  {  //buttons not changed or not enough time elapsed since last change
    return buttonsInputLongPressMask;       //no change to long-presses
  }
    //buttons state changed from 'buttonsInputTrackedMask' and
    // has been steady for BUTTON_DEBOUNCE_TIMEMS
              //set long-press-mask bits based on buttons held and timing:
  if((buttonsInputCurrentMask & BOTH_BUTTONS_MASK) != (byte)0)
  {  //one or both buttons currently pressed
    if(curTimeMs - buttonsInputLastChangeTime >= BUTTON_LONGPRESS_TIMEMS)
    {  //button(s)-held duration enough for long press; set flag(s)
      if((buttonsInputCurrentMask & UP_BUTTON_MASK) != (byte)0)
        buttonsInputLongPressMask |= UP_LONGPRESS_MASK;
      if((buttonsInputCurrentMask & DOWN_BUTTON_MASK) != (byte)0)
        buttonsInputLongPressMask |= DOWN_LONGPRESS_MASK;
      if(clearButtonClickOnLongPressFlag ||
                          (buttonsInputLongPressMask&BOTH_LONGPRESS_MASK) ==
                                                        BOTH_LONGPRESS_MASK)
      {  //flag set or both buttons are long-pressed
        buttonsInputDetectedMask = NO_BUTTONS_MASK;   //clear button presses
      }
    }
  }
  else  //both buttons released
    buttonsInputLongPressMask = 0;          //clear any long-press bits
    //check if any buttons were pressed and have now been released
  if(buttonsInputTrackedMask == NO_BUTTONS_MASK)
  {  //previous state was no buttons pressed
              //don't return new buttons state until both buttons released:
    if(buttonsInputCurrentMask == NO_BUTTONS_MASK)
    {  //both buttons released; updated tracked state
      buttonsInputTrackedMask = buttonsInputCurrentMask;
              //buttons-pressed state is new; accept and return mask
      const byte retMask =             // (including long-press-mask bits)
                       buttonsInputDetectedMask | buttonsInputLongPressMask;
      buttonsInputDetectedMask = NO_BUTTONS_MASK;
      return retMask;
    }
  }
    //previous state was button(s) pressed; wait for no buttons pressed
  buttonsInputTrackedMask = buttonsInputCurrentMask;  //updated tracked state
         //no new button presses, but long-press(es) may be in progress:
  return buttonsInputLongPressMask;
}

//Processes button inputs and returns command-action strings.
// bEnabledFlag:  true for button-actions enabled; false for disabled.
// Returns:  Command-action string, or NULL if none.
char *processButtonInputs(boolean bEnabledFlag)
{
  static byte lastButtonsStateVal = NO_BUTTONS_MASK;
  static unsigned long lastButtonsActionTimeMs = 0;
  static byte newButtonModeValue = BTNFN_NOTSET_MODE;

    //read buttons (if not FREQMHZ_MODE treat long up/down presses like clicks):
  byte buttonsStateVal = readButtonsState(
                            buttonsFunctionModeValue == BTNFN_FREQMHZ_MODE);
  char *cmdStr = NULL;
  if(bEnabledFlag)
  {  //button actions enabled
    const unsigned long curTimeMs = millis();
         //special handling for holding up or down button for MHz value:
    if(buttonsFunctionModeValue == BTNFN_FREQMHZ_MODE &&
               (buttonsStateVal & BOTH_LONGPRESS_MASK) != NO_BUTTONS_MASK &&
           (buttonsStateVal & BOTH_LONGPRESS_MASK) != BOTH_LONGPRESS_MASK &&
                 (buttonsStateVal & BOTH_BUTTONS_MASK) == NO_BUTTONS_MASK &&
            curTimeMs - lastButtonsActionTimeMs >= BUTTON_REPEATINTERVAL_MS)
    {  //one (but not both) buttons held, no presses, and enough time elapsed
              //turn long press into repeated button presses:
      buttonsStateVal = ((buttonsStateVal & UP_LONGPRESS_MASK) != (byte)0) ?
                                          UP_BUTTON_MASK : DOWN_BUTTON_MASK;
    }
    if((buttonsStateVal & BOTH_BUTTONS_MASK) != NO_BUTTONS_MASK)
    {  //new button press detected
      if(!displayRssiEnabledFlag)
      {  //not showing live RSSI on display
        switch(buttonsFunctionModeValue)
        {
          case BTNFN_FREQSCAN_MODE:      //scan and step through active channels
                //simulate commands based on button presses:
            if((buttonsStateVal & BOTH_BUTTONS_MASK) == BOTH_BUTTONS_MASK)
              cmdStr = "M";      //if both buttons then start monitor mode
            else
            {  //one button pressed; do up/down scan and tune
              if((buttonsStateVal & DOWN_BUTTON_MASK) == DOWN_BUTTON_MASK)
                cmdStr = "P";
              else
                cmdStr = "N";
                       //save tuned freq to EEPROM 3 secs after last press
                       // (do this for button presses, but not via serial):
              scheduleDelayedSaveFreqToEeprom(3);
            }
            break;
          case BTNFN_FREQMHZ_MODE:       //increment/decrement freq in MHz
                //simulate commands based on button presses:
            if((buttonsStateVal & BOTH_BUTTONS_MASK) == BOTH_BUTTONS_MASK)
              cmdStr = "A";      //if both buttons then do auto-scan/tune
            else
            {  //one button pressed; increment/decrement frequency by 1 MHz
              if((buttonsStateVal & DOWN_BUTTON_MASK) == DOWN_BUTTON_MASK)
                processOneMHzCommand(false,false);
              else
                processOneMHzCommand(true,false);
              if(!displayConnectedFlag)          //if no display then
                updateActivityIndicator(true);   //indicate "extra" activity
            }
            break;
          default:                       //increment frequency band/chan codes
                //simulate commands based on button presses:
            if((buttonsStateVal & BOTH_BUTTONS_MASK) == BOTH_BUTTONS_MASK)
              cmdStr = "A";      //if both buttons then do auto-scan/tune
            else
            {  //one button pressed; increment freq code channel/band
              if((buttonsStateVal & DOWN_BUTTON_MASK) == DOWN_BUTTON_MASK)
                processIncFreqCodeCommand(true,true,true);
              else
                processIncFreqCodeCommand(false,true,true);
              if(!displayConnectedFlag)          //if no display then
                updateActivityIndicator(true);   //indicate "extra" activity
            }
            break;
        }
        lastButtonsActionTimeMs = curTimeMs;  //mark time of press
      }
      else
      {  //showing live RSSI on display
              //ignore (spurious) press right after long press
        if(curTimeMs - lastButtonsActionTimeMs > 250)
          cmdStr = "#";      //turn off live RSSI on display
      }
    }
    else
    {  //no new button press detected
#if DISP7SEG_ENABLED_FLAG
    if(displayConnectedFlag)
    {  //display is actually wired in
         //handle long-presses of both buttons for live RSSI on display
         // and extra-long-presses of both buttons for button-mode changes:
        if((lastButtonsStateVal & BOTH_LONGPRESS_MASK) != BOTH_LONGPRESS_MASK)
        {  //previous state was not long press of both buttons
          if((buttonsStateVal & BOTH_LONGPRESS_MASK) == BOTH_LONGPRESS_MASK)
          {  //new state is long press of both buttons
                     //if not currently showing live RSSI on display then
                     // show "00" indicator while buttons are long-pressed:
            if(!displayRssiEnabledFlag)
              disp7SegSetOvrAsciiValues('0',true,'0',true,0);
            lastButtonsActionTimeMs = curTimeMs;     //mark time of action
          }
        }
        else
        {  //previous state was long press of both buttons
          if((buttonsStateVal & BOTH_LONGPRESS_MASK) == BOTH_LONGPRESS_MASK)
          {  //ongoing state is long press of both buttons
            if((!displayRssiEnabledFlag) && curTimeMs - lastButtonsActionTimeMs
                                                > BUTTON_EXTRALONGPRESS_TIMEMS)
            {  //not showing RSSI and reached time for button-mode-change action
              if(newButtonModeValue == BTNFN_NOTSET_MODE)
              {  //starting button-mode-change action; start with current value
                newButtonModeValue = buttonsFunctionModeValue;
                lastButtonsActionTimeMs = curTimeMs;    //mark time of action
              }
              else
              {  //button-mode-change action is in progress; time to change
                if(++newButtonModeValue > BTNFN_MODE_MAXVAL)  //inc mode
                  newButtonModeValue = BTNFN_MODE_MINVAL;     //wrap around
                lastButtonsActionTimeMs = curTimeMs;    //mark time of action
              }
              showButtonModeOnDisplay(newButtonModeValue,0);  //show mode
            }
          }
          else
          {  //both buttons were long-pressed and now released
            if(displayRssiEnabledFlag ||
                                      newButtonModeValue == BTNFN_NOTSET_MODE)
            {  //showing RSSI or didn't reach time for button-mode change
              disp7SegClearOvrDisplay();        //clear "00" indicator
              cmdStr = "#";         //toggle showing live RSSI on display
            }
            else
            {  //need to finish button-mode-change action
              if(newButtonModeValue != buttonsFunctionModeValue &&
                                      newButtonModeValue != BTNFN_NOTSET_MODE)
              {  //new button-mode value was triggered; enter it
                buttonsFunctionModeValue = newButtonModeValue;
                                    //show new mode value for a short time:
                showButtonModeOnDisplay(buttonsFunctionModeValue,500);
                                    //save new mode to EEPROM:
                saveButtonModeToEeprom(buttonsFunctionModeValue);
              }
              else  //no new button-mode value
                disp7SegClearOvrDisplay();    //clear button-mode display
            }
            newButtonModeValue = BTNFN_NOTSET_MODE;  //reinit new-mode value
            lastButtonsActionTimeMs = curTimeMs;     //mark time of action
          }
        }
      }
#endif  //DISP7SEG_ENABLED_FLAG
    }
  }
  lastButtonsStateVal = buttonsStateVal;
  return cmdStr;
}

#endif  //BUTTONS_ENABLED_FLAG

//Updates the RSSI value sent to the RSSI (PWM/analog) output pin and
// (possibly) the display.
// rssiVal:  output value in the range 0 (lowest) to 100 (highest).
void updateRssiOutValue(uint16_t rssiVal)
{
#ifdef RSSI_OUT_PIN
  analogWrite(RSSI_OUT_PIN,
                         (int)map(rssiVal,MIN_RSSI_VAL,MAX_RSSI_VAL,0,255));
#endif
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag && displayRssiEnabledFlag)
    showRssiValueOnDisplay(rssiVal);
#endif
}

//Schedules save of tuned-frequency value to EEPROM given number seconds
// after last call this function.  Additional calls within that time will
// delay when the save occurs.
void scheduleDelayedSaveFreqToEeprom(int secs)
{
  delayedSaveFreqToEepromTime = millis() + secs*1000;
  delayedSaveFreqToEepromFlag = true;
}

//Saves given frequency value to EEPROM.
void saveFreqValToEeprom(uint16_t freqVal)
{
  writeWordToEeprom(EEPROM_ADRW_FREQ,freqVal);
}

//Saves the current frequency value to EEPROM (if changed).
void saveCurrentFreqToEeprom()
{
  const uint16_t curFreqVal = currentTunerFreqMhzOrCode;
  if(curFreqVal != lastEepromFreqInMhzOrCode)
  {  //different from saved value
    saveFreqValToEeprom(curFreqVal);
    lastEepromFreqInMhzOrCode = curFreqVal;
  }
}

//Loads and returns frequency value from EEPROM.
uint16_t loadFreqValFromEeprom()
{
  const uint16_t fVal = readWordFromEeprom(EEPROM_ADRW_FREQ);
  if(fVal == (uint16_t)0xFFFF)
  {  //no value saved; use default value
    lastEepromFreqInMhzOrCode = DEFAULT_FREQ_MHZ;
    return DEFAULT_FREQ_MHZ;
  }
  lastEepromFreqInMhzOrCode = fVal;     //keep track of saved value
  return fVal;
}

//Set tuner to frequency value from EEPROM (or default if never saved).
void setChanToFreqValFromEeprom()
{
  uint16_t freqVal = loadFreqValFromEeprom();
  if(freqVal < FREQ_CODEWORD_CHECKVAL)
  {  //loaded frequency value not code word; check range
    if(freqVal < MIN_CHANNEL_MHZ || freqVal > MAX_CHANNEL_MHZ)
      freqVal = DEFAULT_FREQ_MHZ;      //if out of range then use default
    const uint16_t codeVal = freqInMhzToFreqCode(freqVal,NULL);
    if(codeVal != (uint16_t)0)
    {  //freq code-word value found for freq in MHz
      freqVal = codeVal;                    //use code-word value
      saveFreqValToEeprom(freqVal);         //update saved value
    }
  }
  setTunerChannelToFreq(freqVal);           //tune to channel
}

//Saves given button-mode value to EEPROM.
void saveButtonModeToEeprom(byte modeVal)
{
  writeByteToEeprom(EEPROM_ADRB_BTNMODE,modeVal);
}

//Loads and returns button-mode value from EEPROM.
byte loadButtonModeFromEeprom()
{
  byte modeVal = readByteFromEeprom(EEPROM_ADRB_BTNMODE);
  if(modeVal < BTNFN_MODE_MINVAL || modeVal > BTNFN_MODE_MAXVAL)
  {  //no value saved; save and return default value
    modeVal = displayConnectedFlag ?   // (different default if no display)
                                 BTNFN_MODE_WDISPDEF : BTNFN_MODE_NODISPDEF;
    saveButtonModeToEeprom(modeVal);
  }
  return modeVal;
}

//Saves current min/max-raw-RSSI values for scaling to EEPROM.
void saveRssiMinMaxValsToEeprom()
{
  if(autoRssiCalibShowOutputFlag && serialEchoFlag)
  {  //debug output (and serial echo) enabled
    Serial.print(F("[New RSSI calib min/max scaling values: "));
    Serial.print((int)getRx5808RawRssiMinVal());
    Serial.print(',');
    Serial.print((int)getRx5808RawRssiMaxVal());
    Serial.println(']');
  }
  writeWordToEeprom(EEPROM_ADRW_RSSIMIN,getRx5808RawRssiMinVal());
  writeWordToEeprom(EEPROM_ADRW_RSSIMAX,getRx5808RawRssiMaxVal());
}

//Loads and stores current min/max-raw-RSSI values for scaling from EEPROM.
void loadRssiMinMaxValsFromEeprom()
{
  uint16_t minVal = readWordFromEeprom(EEPROM_ADRW_RSSIMIN);
  uint16_t maxVal = readWordFromEeprom(EEPROM_ADRW_RSSIMAX);
  if(minVal == (uint16_t)0xFFFF || maxVal == (uint16_t)0xFFFF ||
                                                           minVal >= maxVal)
  {  //values not saved or are invalid; use and save default values
    minVal = DEF_RAWRSSI_MIN;
    maxVal = DEF_RAWRSSI_MAX;
    writeWordToEeprom(EEPROM_ADRW_RSSIMIN,minVal);
    writeWordToEeprom(EEPROM_ADRW_RSSIMAX,maxVal);
  }
  setRx5808RawRssiMinMax(minVal,maxVal);
}

//Saves given auto RSSI calibration enabled flag value to EEPROM.
void saveAutoRssiCalFlagToEeprom(boolean flagVal)
{
  writeByteToEeprom(EEPROM_ADRB_AUTOCAL, flagVal ? (byte)1 : (byte)0);
}

//Loads and returns auto RSSI calibration enabled flag value from EEPROM.
boolean loadAutoRssiCalFlagFromEeprom()
{
  byte val = readByteFromEeprom(EEPROM_ADRB_AUTOCAL);
  boolean retFlag;
  if(val > (byte)1)
  {  //no valid value saved; save and return default value
    retFlag = DEF_AUTOCAL_FLAG;
    saveAutoRssiCalFlagToEeprom(retFlag);
  }
  else
    retFlag = (val != (byte)0) ? true : false;
  return retFlag;
}

//Saves given the RX5808 minimum-tune time (in ms) value to EEPROM.
void saveMinTuneTimeMsToEeprom(byte timeVal)
{
  writeByteToEeprom(EEPROM_ADRB_MINTUNEMS,timeVal);
}

//Loads and returns the RX5808 minimum-tune time (in ms) value from EEPROM.
byte loadMinTuneTimeMsFromEeprom()
{
  byte timeVal = readByteFromEeprom(EEPROM_ADRB_MINTUNEMS);
  if(timeVal == (byte)0xFF)
  {  //no value saved; save and return default value
    timeVal = RX5808_MIN_TUNETIME;
    saveMinTuneTimeMsToEeprom(timeVal);
  }
  return timeVal;
}

//Saves the given Unit-ID string to EEPROM.
void saveUnitIdToEeprom(const char *str)
{
  writeStringToEeprom(EEPROM_ADRS_UNITID,str,EEPROM_FLEN_UNITID);
}

//Determines if Unit-ID string from EEPROM is empty.
// Returns true if Unit-ID string from EEPROM is empty (or field is
// uninitialized).
boolean isUnitIdFromEepromEmpty()
{
  return isStringFromEepromEmpty(EEPROM_ADRS_UNITID);
}

//Displays Unit-ID string from EEPROM, sending characters to serial port.
void showUnitIdFromEeprom()
{
  showStringFromEeprom(EEPROM_ADRS_UNITID,EEPROM_FLEN_UNITID);
}

//Saves the list of values entered via the 'L' command to EEPROM.
void saveListFreqsMHzArrToEeprom()
{
  writeUint16ArrayToEeprom(EEPROM_ADRA_FREQLIST,EEPROM_FLEN_FREQLIST,
                                      listFreqsMHzArr,listFreqsMHzArrCount);
}

//Loads the list of values entered via the 'L' command from EEPROM.
void loadListFreqsMHzArrFromEeprom()
{
  listFreqsMHzArrCount = readUint16ArrayFromEeprom(EEPROM_ADRA_FREQLIST,
                                      listFreqsMHzArr,LISTFREQMHZ_ARR_SIZE);
}

//Sets all EEPROM values to defaults (by setting all used bytes to 0xFF).
void setEepromToDefaultsValues()
{
  Serial.println(F(" Setting configuration to default values"));
  for(int addr=0; addr<EEPROM_USED_DATASIZE; ++addr)
    writeByteToEeprom(addr,(byte)0xFF);
         //write EEPROM integrity-check value:
  writeWordToEeprom(EEPROM_ADRW_CHECKWORD,EEPROM_CHECK_VALUE);
}

//Checks if the EEPROM integrity-check value matches and sets all EEPROM
// values to defaults if not.
void checkEepromIntegrity()
{
  if(readWordFromEeprom(EEPROM_ADRW_CHECKWORD) != EEPROM_CHECK_VALUE)
    setEepromToDefaultsValues();
}

//If the 7-segment displays are enabled and connected then this function
// does nothing; otherwise this function flickers the status LED (D13)
// to indicate activity.
// activityFlag:  true to show extra activity via the LED.
void updateActivityIndicator(boolean activityFlag)
{
  static unsigned long nextIndicatorTimeMs = 0;
  static boolean lastActivityFlag = false;

  const unsigned long curTimeMs = millis();
#if DISP7SEG_ENABLED_FLAG
  if(displayConnectedFlag)
  {  //display is wired in
    if(activityFlag)
    {  //flag set for "extra" activity
      if(curTimeMs >= nextIndicatorTimeMs)
      {  //not too soon after last display; flash indicator
        disp7SegSetOvrAsciiValues('8',true,'8',true,25);
      }
         //update indicator time so repeated fast occurrences are ignored
         // (this prevents display from being overly disturbed):
      nextIndicatorTimeMs = curTimeMs + 100;
    }
    return;
  }
#endif
  const boolean curStateFlag = (digitalRead(NODISP_ACTIVITY_PIN) != LOW);
  boolean newStateFlag;
  if(activityFlag)
  {  //indicate "extra" activity
    if(!lastActivityFlag)
    {  //activityFlag state is new
      newStateFlag = true;
      nextIndicatorTimeMs = curTimeMs + 100;     //set next change time
    }
    else
    {  //activityFlag state looks same as last time (constant activity)
      if(curTimeMs >= nextIndicatorTimeMs)
      {  //time reached to toggle indicator output
        newStateFlag = !curStateFlag;    //change state
        nextIndicatorTimeMs = curTimeMs + 35;    //show fast pulsing
      }
      else  //time not reached; no change
        newStateFlag = curStateFlag;
    }
  }
  else
  {  //indicate normal activity
    if(curTimeMs >= nextIndicatorTimeMs)
    {  //time reached to toggle indicator output
      newStateFlag = !curStateFlag;    //change state
              //setup next change time (longer if "not active" state):
      nextIndicatorTimeMs = curTimeMs + (newStateFlag ? 50 : 950);
    }
    else  //time not reached; no change
      newStateFlag = curStateFlag;
  }
  if(newStateFlag != curStateFlag)     //if new state then update output
    digitalWrite(NODISP_ACTIVITY_PIN, (newStateFlag ? HIGH : LOW));
  lastActivityFlag = activityFlag;     //track activity state
}

//Reads and averages a set of RSSI samples for the currently-tuned channel.
// Returns:  An averaged RSSI value from MIN_RSSI_VAL to MAX_RSSI_VAL.
uint16_t readRssiValue()
{
  uint16_t rawVal = readRawRssiValue();
  if(autoRssiCalibEnabledFlag)         //if auto-calib enabled then
    processAutoRssiCalValue(rawVal);   //process received value
  return scaleRawRssiValue(rawVal);    //scale MIN_RSSI_VAL to MAX_RSSI_VAL
}

//Does auto RSSI calibration with given value.
// rawRssiVal:  Raw RSSI value.
void processAutoRssiCalValue(uint16_t rawVal)
{
         //when there are multiple occurrences of out-of-range values
         // within a short time, adjust the min or max scaling value
         // in the direction of the out-of-range value
         // (for RSSI values < rawRssiMin only accept if tuner frequency
         //  is in a "good" range so low RSSI values from lower frequencies
         //  don't skew the rawRssiMin for the whole band)
  if(((rawVal < getRx5808RawRssiMinVal() && currentTunerFreqInMhz > 5640 &&
                                            currentTunerFreqInMhz < 5950) ||
                                       rawVal > getRx5808RawRssiMaxVal()) &&
                     rawVal >= CHK_RAWRSSI_MIN && rawVal <= CHK_RAWRSSI_MAX)
  {  //RSSI value outside current min/max range (but within check range)
    if(autoRssiCalibCounterValue == (byte)0)
    {  //this is the "first" out-of-range value
      autoRssiCalibMarkedTime = millis();        //save value for later
      autoRssiCalibCounterValue = (byte)1;
    }
    else if(millis() <= autoRssiCalibMarkedTime + 5000)
    {  //this "next" out-of-range value is not too long after "first" one
      if(autoRssiCalibCounterValue >= 3)
      {  //enough out-of-range values in a row
        autoRssiCalibCounterValue = (byte)0;     //reset counter
        uint16_t minVal = getRx5808RawRssiMinVal();
        uint16_t maxVal = getRx5808RawRssiMaxVal();
        int adjVal;
        if(rawVal < minVal)
        {  //value is less than current minimum; decrease min scaling value
              //use "margin" to keep raw-RSSI-min from getting to low:
          int margin = (minVal-75) / 6;     // (less margin when smaller)
          if(minVal < rawVal + margin)      //if min would get too low
            return;                         // then don't chage it
          if((adjVal=(minVal-rawVal)/5) <= 0)    //calc adjustment amount
            adjVal = 1;                          //make it at least 1
          minVal -= adjVal;
          if(minVal < CHK_RAWRSSI_MIN || minVal > CHK_RAWRSSI_MAX)
            return;          //if outside range then don't change
        }
        else
        {  //value is more than current maximum; increase max scaling value
          if((adjVal=(rawVal-maxVal)/5) <= 0)    //calc adjustment amount
            adjVal = 1;                          //make it at least 1
          maxVal += adjVal;
          if(maxVal < CHK_RAWRSSI_MIN || maxVal > CHK_RAWRSSI_MAX)
            return;          //if outside range then don't change
        }
        setRx5808RawRssiMinMax(minVal,maxVal);   //update values
        saveRssiMinMaxValsToEeprom();            //save persistent values
      }
      else
        ++autoRssiCalibCounterValue;
    }
    else  //too long after "first" value
      autoRssiCalibCounterValue = (byte)0;       //reset counter
  }
}

#if DISP7SEG_ENABLED_FLAG

//Shows program-version code on 7-segment displays.
void showProgramVersionOnDisplay()
{
  const char *revStr = PROG_VERSION_STR;
  const int sLen = strlen(revStr);
  if(sLen > 0)
  {  //at least on character available
    int p = 0;
    const char leftCh = revStr[p];          //first char
    boolean dpFlag = false;
    if(++p < sLen && revStr[p] == '.')
    {  //decimal point found after first character
      dpFlag = true;         //setup to show decimal point on display
      ++p;
    }
    const char rightCh = (p < sLen) ? revStr[p] : ' ';     //second char
         //show version digits as part of initial display:
    disp7SegSetInitAsciiValues('A',false,'r',false,
                                           leftCh,dpFlag,rightCh,false,500);
  }
}

//Shows current tuner channel on display.
void showTunerChannelOnDisplay()
{
         //get two-character frequency-code value (if any):
  const uint16_t codeVal = getCurrentFreqCodeWord();
  uint16_t word1;
  int count1;
  if(codeVal > (uint16_t)0)
  {  //frequency-code value available; generate bitmask word for code chars
    word1 = disp7SegConvAsciiCharsToWord((char)(codeVal>>(uint16_t)8),false,
                                      (char)(codeVal&(uint16_t)0x7F),false);
    count1 = 10;   //set display time for code value
  }
  else
  {  //no frequency-code value; generate bitmask for blank displays
    word1 = disp7SegConvAsciiCharsToWord(' ',false,' ',false);
    count1 = 2;    //set display time for code value (shorter for none)
  }

         //convert frequency value to text characters:
  itoa((int)getCurrentFreqInMhz(),itoaBuff,10);
  int count2,count3;
  uint16_t word2,word3;
  if(strlen(itoaBuff) >= 4)
  {  //enough text characters
         //generate bitmask word for first two digits (and both DPs):
    word2 = disp7SegConvAsciiCharsToWord(itoaBuff[0],true,itoaBuff[1],true);
    count2 = 8;         //set display time for first two frequency digits
         //generate bitmask word for second two digits (and 2nd DP):
    word3 = disp7SegConvAsciiCharsToWord(itoaBuff[2],false,itoaBuff[3],true);
    count3 = 10;        //set display time for second two frequency digits
  }
  else
  {  //not enough characters; no output of frequency digits
    count2 = count3 = 0;
    word2 = word3 = (uint16_t)0;
  }
         //enter bitmask words into display array:
  disp7SegEnterToDisplayWordsArr(word1,count1,word2,count2,word3,count3);
}

//Shows given numeric value on 7-segment displays.
// dispVal:  Value to be shown (0-99).
// leftDpFlag:  true to show decimal point on left display.
// rightDpFlag:  true to show decimal point on right display.
// dispTimeMs:  Display time in ms, or 0 for indefinite.
void showNumericValueOnDisplay(uint16_t dispVal, boolean leftDpFlag,
                                        boolean rightDpFlag, int dispTimeMs)
{
  char leftCh,rightCh;
  if(dispVal <= 99)
  {  //value not too large
    if(dispVal <= 9)
    {  //leading zero and one digit
      leftCh = '0';
      rightCh = (char)dispVal + '0';
    }
    else
    {  //two digits
      leftCh = (char)(dispVal/10) + '0';
      rightCh = (char)(dispVal%10) + '0';
    }
  }
  else  //value too large; show overflow indicator
    leftCh = rightCh = 'o';
              //send characters to display:
  disp7SegSetOvrAsciiValues(leftCh,leftDpFlag,
                                            rightCh,rightDpFlag,dispTimeMs);
}

//Shows given RSSI value on 7-segment displays.
// rssiVal:  Value in the range 0 (lowest) to 100 (highest).
void showRssiValueOnDisplay(uint16_t rssiVal)
{
  static unsigned long nextUpdateTimeMs = 0;
  static uint16_t rssiAvgrTotal = 0;
  static uint16_t lastRssiValue = 0;
  static byte updateAvgrCounter = 0;
  static byte dpPatternMask = 0;

  rssiAvgrTotal += rssiVal;            //add to total for average
  if(++updateAvgrCounter >= 10)
  {  //only update displayed value via average every 10th time
    lastRssiValue = rssiAvgrTotal / (uint16_t)10;
    updateAvgrCounter = rssiAvgrTotal = 0;
  }
  const unsigned long curTimeMs = millis();
  if(curTimeMs >= nextUpdateTimeMs)
  {  //time reached to update display
    nextUpdateTimeMs = curTimeMs + 50;      //set next update time
    if(dpPatternMask == (byte)0)            //if first or pattern done then
      dpPatternMask = (byte)0b00010010;     //enter pattern value
    else                     //rotate to next pair of bits in pattern:
      dpPatternMask = dpPatternMask >>= (byte)2;
              //send value to display, with DP values from bitmask
              // (make 100 be displayed as 99):
    showNumericValueOnDisplay(((lastRssiValue != 100) ? lastRssiValue : 99),
                                       ((dpPatternMask&(byte)2) != (byte)0),
                                ((dpPatternMask&(byte)1) != (byte)0), 1000);
  }
}

#if BUTTONS_ENABLED_FLAG

//Shows given button-mode value on 7-segment displays.
// bModeVal:  Value to display.
// dispTimeMs:  Display time in ms, or 0 for indefinite.
void showButtonModeOnDisplay(byte bModeVal, int dispTimeMs)
{
  disp7SegSetOvrAsciiValues('=',false,
                                     ((char)bModeVal+'0'),false,dispTimeMs);
}

#endif  //BUTTONS_ENABLED_FLAG
#endif  //DISP7SEG_ENABLED_FLAG
