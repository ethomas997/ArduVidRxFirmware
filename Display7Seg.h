//Display7Seg.h:  Header file for pair of 7-segment displays support code.
//
//  12/3/2016 -- [ET]
//

#ifndef DISPLAY7SEG_H_
#define DISPLAY7SEG_H_

    //hardware pin mapping for display segments:
#define DISP7SEG_A_PIN 6
#define DISP7SEG_B_PIN 7
#define DISP7SEG_C_PIN 8
#define DISP7SEG_D_PIN 9
#define DISP7SEG_E_PIN A0
#define DISP7SEG_F_PIN A1
#define DISP7SEG_G_PIN A2
#define DISP7SEG_DP_PIN 13

    //hardware pins for selecting left or right digit:
#define DISP7SEG_SELLEFT_PIN A3
#define DISP7SEG_SELRIGHT_PIN A4

void disp7SegSetup();
void disp7SegShutdown();
uint16_t disp7SegConvAsciiCharsToWord(char leftCh, boolean leftDpFlag,
                                         char rightCh, boolean rightDpFlag);
void disp7SegSetInitAsciiValues(char leftCh1, boolean leftDpFlag1,
                                        char rightCh1, boolean rightDpFlag1,
                                          char leftCh2, boolean leftDpFlag2,
                                        char rightCh2, boolean rightDpFlag2,
                                                            int dispTimeMs);
void disp7SegSetOvrAsciiValues(char leftCh, boolean leftDpFlag, char rightCh,
                                       boolean rightDpFlag, int dispTimeMs);
void disp7SegSetOvrAsciiViaWord(uint16_t wordVal, int dispTimeMs);
void disp7SegSetOvrShowDashes(int dispTimeMs);
void disp7SegClearOvrDisplay();
void disp7SegEnterToDisplayWordsArr(uint16_t word1, int count1,
                    uint16_t word2, int count2, uint16_t word3, int count3);
boolean disp7SegTestDisplayConnected();

#endif /* DISPLAY7SEG_H_ */
