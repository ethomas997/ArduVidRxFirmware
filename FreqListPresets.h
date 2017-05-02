//FreqListPresets.h:  Header file for frequency-list-presets manager.
//
// 12/17/2016 -- [ET]
//

#ifndef FREQLISTPRESETS_H_
#define FREQLISTPRESETS_H_

int freqListPresetShowForName(const char *nameStr);
int freqListPresetLoadByName(const char *nameStr, uint16_t *freqArr,
                                                              int maxCount);
void freqListPresetShowAllSets();

#endif /* FREQLISTPRESETS_H_ */
