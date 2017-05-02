// Rx5808Fns.h

#ifndef RX5808FNS_H_
#define RX5808FNS_H_

#define MIN_CHANNEL_MHZ 4000      //min MHz value for commands
#define MAX_CHANNEL_MHZ 7000      //max MHz value for commands

// Receiver Module version
// used for tuning time
#define rx5808
//#define rx5880

#define MIN_RSSI_VAL 0       //min/max range for RSSI values
#define MAX_RSSI_VAL 100
// number of analog rssi reads to average for the current check.
#define RSSI_READS 20

// RSSI default raw range
//#define RAW_RSSI_MIN 90
//#define RAW_RSSI_MAX 220
// 75% threshold, when channel is printed in spectrum
//#define RSSI_SEEK_FOUND 75
// 80% under max value for RSSI
//#define RSSI_SEEK_TRESHOLD 80
// scan loops for setup run
//#define RSSI_SETUP_RUN 3

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#if USE_LBAND_FLAG
    #define CHANNEL_MAX_INDEX 47
#else
    #define CHANNEL_MAX_INDEX 39
#endif
#define LBAND_FIRST_INDEX 40

#ifdef rx5808
    // rx5808 module need >20ms to tune.
    // 25 ms will do a 40 channel scan in 1 second.
    // 35 ms will do a 40 channel scan in 1.4 seconds.
    #define RX5808_MIN_TUNETIME 35
#endif
#ifdef rx5880
    // rx5880 module needs >30ms to tune.
    // 35 ms will do a 40 channel scan in 1.4 seconds.
    #define RX5808_MIN_TUNETIME 35
#endif

    //lower than this is freq in MHz; otherwise code word (i.e., "E8"):
#define FREQ_CODEWORD_CHECKVAL ((((uint16_t)' ')<<(uint16_t)8)+' ')

void rx5808setup();
void setRx5808RawRssiMinMax(uint16_t minVal, uint16_t maxVal);
uint16_t getRx5808RawRssiMinVal();
uint16_t getRx5808RawRssiMaxVal();
void setRx5808MinTuneTimeMs(uint8_t val);
uint8_t getRx5808MinTuneTimeMs();
boolean isPriRx5808RssiInPinInUse();
uint16_t freqMhzToRegVal(uint16_t freqInMhz);
uint16_t regValToFreqMhz(uint16_t regVal);
void setChannelByFreq(uint16_t freqInMhz);
uint16_t getChannelFreqTableEntry(int idx);
uint16_t getChannelRegTableEntry(int idx);
uint8_t getChannelSortTableEntry(int idx);
int getIdxForFreqInMhz(uint16_t freqVal);
void waitRssiReady();
uint16_t readRawRssiValue();
uint16_t scaleRawRssiValue(uint16_t rawRssiVal);
uint16_t sampleRawRssiValue();
boolean isLBandChannelIndex(int idx);
uint16_t freqCodeCharsToFreqInMhz(char bandCh, char chanCh);
uint16_t freqCodeWordToFreqInMhz(uint16_t codeWordVal);
uint16_t freqCodeStrToCodeWord(const char *codeStr);
uint16_t freqIdxToFreqCode(int freqIdx, char *outStr);
uint16_t freqInMhzToFreqCode(uint16_t freqVal, char *outStr);
uint16_t freqInMhzToNearestFreqCode(uint16_t freqVal, boolean upFlag,
                                                              char *outStr);
uint16_t incDecFreqCodeValue(uint16_t codeWordVal, boolean bandFlag,
                                                            boolean upFlag);

#endif /* RX5808FNS_H_ */
