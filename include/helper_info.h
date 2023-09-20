#ifndef HELPER_INFO_H
#define HELPER_INFO_H

#include "all.h"

#define TEXTSTARTX 20
#define TEXTSTARTY 20
#define TEXTGAP 15
#define TEXTGAPHELP 20
#define NEWLINE 20

extern double FuelToBurn;
extern double KE;
extern double EstimatedTimeToBurnSuicide;
extern bool SuicideBurnStarted;
extern bool Landed;
extern double EnergyToBurn;
static bool TimerStarted = false;
static bool TimerStopped = false;;

typedef void (*VoidFunction)();

void DisplayTransferStatus();

void HelperInfoDrawer(VoidFunction HelpfulfInformation);

VoidFunction HelpfulInformation(COPILOT_ACTION CurrentAction);

#endif