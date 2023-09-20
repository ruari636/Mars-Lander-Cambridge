#ifndef HELPER_INFO_H
#define HELPER_INFO_H

#include "all.h"

#define TEXTSTARTX 20
#define TEXTSTARTY 20
#define TEXTGAP 15
#define NEWLINE 20

typedef void (*VoidFunction)();

void DisplayTransferStatus();

void HelperInfoDrawer(VoidFunction HelpfulfInformation);

VoidFunction HelpfulInformation(COPILOT_ACTION CurrentAction);

#endif