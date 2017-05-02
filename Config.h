//Config.h:  Configuration file for ArduVidRx.
//
//  4/23/2017 -- [ET]
//

#ifndef CONFIG_H_
#define CONFIG_H_

#define DISP7SEG_ENABLED_FLAG true     //true to enable 7-segment displays
#define BUTTONS_ENABLED_FLAG true      //true to enable button inputs
#define USE_LBAND_FLAG true            //true to scan for 'L'-band frequencies

#define DEFAULT_FREQ_MHZ 5800          //default freq if none saved in EEPROM
#define SERIAL_BAUDRATE 115200         //serial-port baud rate

#define DEF_MONITOR_INTERVAL_SECS 5    //default time for 'M' commend
#define BUTTON_REPEATINTERVAL_MS 25    //speed of up/down-MHz via held buttons
              //for commands with rescans ('N','P','M'), always rescan
              // if this much time has elapsed since last scan:
#define NEXT_CHAN_RESCANSECS 120L

#define DEF_MIN_RSSI_LEVEL 30          //min RSSI for "active" channel
              //minimum spacing when squelching adjacent channels
              // for 'S','N','P','M' commands (but not 'F' command):
#define ADJ_CHAN_MHZ 30

#define RSSI_SAMPAVG_COUNT 50          //averaging size for RSSI out

#define DEF_RAWRSSI_MIN 180            //min-raw-RSSI value for scaling
#define DEF_RAWRSSI_MAX 200            //max-raw-RSSI value for scaling
#define DEF_AUTOCAL_FLAG true          //default auto RSSI calib enabled flag
#define CHK_RAWRSSI_MIN 20             //minimum raw-RSSI check value
#define CHK_RAWRSSI_MAX 1000           //maximum raw-RSSI check value

              //output pin for activity-indicator LED if no 7-segment display:
#define NODISP_ACTIVITY_PIN ((unsigned char)13)       //13 == D13

              //hardware I/O mapping:
#define RSSI_PRI_PIN A7                //RSSI input from RX5808 (primary)
#define RSSI_SEC_PIN A6                //RSSI input if no signal on primary
#define UP_BUTTON_PIN 2                //UP button
#define DOWN_BUTTON_PIN 3              //DOWN button
#define RSSI_OUT_PIN 4                 //analog RSSI output (PWM)
#define EXTRA_OUT_PIN 5                //extra output (maybe addressable LEDs)
#define RX5808_DATA_PIN 10             //DATA output line to RX5808 module
#define RX5808_SEL_PIN 11              //CLK output line to RX5808 module
#define RX5808_CLK_PIN 12              //SEL output line to RX5808 module

              //unused inputs to set to INPUT_PULLUP (can be commented out):
#define PULLUP_1_PIN A5                //pin to set to INPUT_PULLUP
//#define PULLUP_2_PIN A6              // (note: no A6 pullup on ATMega328)

              //pins to be setup as outputs if no 7-segment display:
#define NODISP_PRIVSEL_PIN A1          //set low if RSSI_PRI_PIN in use
#define NODISP_SECVSEL_PIN A0          //set low if RSSI_SEC_PIN in use

//also see "Display7Seg.h" for display-pin mappings

              //bitmask codes for button inputs:
#define NO_BUTTONS_MASK ((byte)0)
#define UP_BUTTON_MASK ((byte)1)
#define DOWN_BUTTON_MASK ((byte)2)
#define BOTH_BUTTONS_MASK ((byte)3)
#define UP_LONGPRESS_MASK ((byte)16)
#define DOWN_LONGPRESS_MASK ((byte)32)
#define BOTH_LONGPRESS_MASK ((byte)48)
#define BUTTON_DEBOUNCE_TIMEMS 50
#define BUTTON_LONGPRESS_TIMEMS 1000        //long press for show-RSSI
#define BUTTON_EXTRALONGPRESS_TIMEMS 1500   //extra time for mode change

              //button-function modes:
#define BTNFN_FREQCODE_MODE ((byte)1)  //increment frequency band/chan codes
#define BTNFN_FREQSCAN_MODE ((byte)2)  //scan and step through active channels
#define BTNFN_FREQMHZ_MODE ((byte)3)   //increment/decrement freq in MHz
#define BTNFN_NOTSET_MODE ((byte)0)    //not-set value used by button handler
#define BTNFN_MODE_MINVAL BTNFN_FREQCODE_MODE
#define BTNFN_MODE_MAXVAL BTNFN_FREQMHZ_MODE
#define BTNFN_MODE_WDISPDEF BTNFN_FREQCODE_MODE  //default mode if display
#define BTNFN_MODE_NODISPDEF BTNFN_FREQSCAN_MODE //default mode if no display

#endif /* CONFIG_H_ */
