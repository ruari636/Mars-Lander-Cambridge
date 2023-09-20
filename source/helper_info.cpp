#include "helper_info.h"

extern int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
extern double FuelToBurn;

void NothingToShow()
{}

bool InsufficientFuelText()
{
    bool insufficient = abs(FuelToBurn) > fuel;
    if (insufficient)
    {
        glRasterPos2f(TEXTSTARTX, TEXTSTARTY);
        string ss = "Insufficient Fuel. Unable to Initiate Manouvre.";
        for (int i = 0; i < ss.length(); i++) glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, ss[i]);
    }
    return insufficient;
}

void HelperInfoDrawer(VoidFunction HelpfulfInformation)
{
    glColor3f(1.0, 1.0, 1.0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, view_width, 0, view_height, -1.0, 1.0); 
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    HelpfulfInformation();

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void DisplayTransferStatus()
{
  extern double AngleToStartBurn;
  double AngFunc = CalculateAngleXY(position, vector3d(), MoonPos);
  int curYpos = TEXTSTARTY;

  if (!InsufficientFuelText())
  {
    glut_print(TEXTSTARTX, view_height-curYpos, "Relative Angle to Start Transfer - " + to_string(AngleToStartBurn)); curYpos += TEXTGAP;
    glut_print(TEXTSTARTX, view_height-curYpos, "Current Angle - " + to_string(AngFunc)); curYpos += TEXTGAP;
    glut_print(TEXTSTARTX, view_height-curYpos, "Fuel to Burn - " + to_string(abs(FuelToBurn))); curYpos += TEXTGAP;
  }
}

VoidFunction HelpfulInformation(COPILOT_ACTION CurrentAction)
{
    switch (CurrentAction)
    {
        case GOTOMOON: case BIIMPULSIVEMOONTRANSFER:
            return DisplayTransferStatus;
            break;
        default:
            return NothingToShow;
    }
}