#include "helper_info.h"

extern int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
int curYpos;
double EnergyToBurn;
double StartTimer;
double StopTimer;
double EstimatedTimeToBurnSuicideSave;
bool RadiusAdded = true;

void ResetHelperInfo()
{
    StartTimer = 0; StopTimer = 0;
    TimerStarted = false; TimerStopped = false;
}

void glut_print_helper (float x, float y, string s)
  // Prints string at location (x,y) in a bitmap font
{
  unsigned short i;

  glRasterPos2f(x, y);
  for (i = 0; i < s.length(); i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s[i]);
}

void glut_print_helper_medium (float x, float y, string s)
  // Prints string at location (x,y) in a bitmap font
{
  unsigned short i;

  glRasterPos2f(x, y);
  for (i = 0; i < s.length(); i++) glutBitmapCharacter(GLUT_BITMAP_9_BY_15, s[i]);
}

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
  extern double MoonApproachPerigee;
  double AngFunc = CalculateAngleXY(position, vector3d(), MoonPos);

  if (!InsufficientFuelText())
  {
    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Relative Angle to Start Transfer - " + to_string(AngleToStartBurn)); curYpos += TEXTGAPHELP;
    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Current Angle - " + to_string(AngFunc)); curYpos += TEXTGAPHELP;
    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Fuel to Burn - " + to_string(abs(FuelToBurn))); curYpos += TEXTGAPHELP;
    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Apogee aimed for - " + to_string(ApogeeHeight)); curYpos += TEXTGAPHELP;
  }
}

void KEandEstimatedSuicideBurnWork()
{
    if (SuicideBurnStarted)
    {
        if (!TimerStarted)
        {
            StartTimer = simulation_time;
        }
        TimerStarted = true;
    }
    if (!Landed || TimerStopped)
    {
        glut_print_helper(TEXTSTARTX, view_height-curYpos, "Current KE : ");
        glut_print_helper(TEXTSTARTX + 400, view_height-curYpos, to_string((int)KE)); curYpos += TEXTGAPHELP;
        if (Altitude < MAXSUICIDEBURNCHECKHEIGHT * 2)
        {
            glut_print_helper(TEXTSTARTX, view_height-curYpos, "Estimated resistive work available : ");
            glut_print_helper(TEXTSTARTX + 400, view_height-curYpos,
                                 to_string((int)EnergyToBurn)); curYpos += TEXTGAPHELP;
            if (!TimerStarted)
            {
                if (Altitude < 2000) {
                    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Estimated time to landing : "); 
                    glut_print_helper(TEXTSTARTX + 400, view_height-curYpos, to_string(EstimatedTimeToBurnSuicide)); curYpos += TEXTGAPHELP; 
                }
                EstimatedTimeToBurnSuicideSave = EstimatedTimeToBurnSuicide;
            }
            else
            {
                    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Estimated time to landing : "); 
                    glut_print_helper(TEXTSTARTX + 400, view_height-curYpos, to_string(EstimatedTimeToBurnSuicideSave)); curYpos += TEXTGAPHELP;                
            }
        }
        if (TimerStarted)
        {
            glut_print_helper(TEXTSTARTX, view_height-curYpos, "Time since suicide burn started : ");
            glut_print_helper(TEXTSTARTX + 400, view_height-curYpos,
                                 to_string(simulation_time - StartTimer)); curYpos += TEXTGAPHELP;
        }
        if (TimerStopped)
        {
            glut_print_helper(TEXTSTARTX, view_height-curYpos, "Time since suicide burn : ");
            glut_print_helper(TEXTSTARTX + 400, view_height-curYpos,
                                 to_string(StopTimer - StartTimer)); curYpos += TEXTGAPHELP;
        }
    }
    else
    {
        if (!TimerStopped)
        {
            StopTimer = simulation_time;
        }
        TimerStopped = true;
        TimerStarted = false;
    }
}

void display_input_interface (void)
{
  int curXpos = TEXTSTARTX;  

    glut_print_helper(TEXTSTARTX, view_height-curYpos, "Press R to add radius of body to inputs");
    if (RadiusAdded) { glut_print_helper(TEXTSTARTX, view_height-curYpos - TEXTGAPHELP, "Radius Included"); }

    glut_print_helper_medium(curXpos, TEXTSTARTY + NEWLINE, "New Apogee Alt: ");
    glut_print_helper_medium(curXpos + 5, TEXTSTARTY, " .");
    for (int i = 0; i < INPUTRESOLUTION; i++)
    {
        glut_print_helper_medium(curXpos, TEXTSTARTY, to_string(CustomOrbitInput[i]));
        curXpos += 15;
    }
    glut_print_helper_medium(curXpos, TEXTSTARTY, "x10 " + to_string(CustomOrbitInput[INPUTRESOLUTION]));

    curXpos += 90;
    glut_print_helper_medium(curXpos, TEXTSTARTY + NEWLINE, "New Perigee Alt: ");
    glut_print_helper_medium(curXpos + 5, TEXTSTARTY, " .");
    for (int i = INPUTRESOLUTION + 1; i < 2 * INPUTRESOLUTION + 1; i++)
    {
        glut_print_helper_medium(curXpos, TEXTSTARTY, to_string(CustomOrbitInput[i]));
        curXpos += 15;
    }
    glut_print_helper_medium(curXpos, TEXTSTARTY, "x10 " + to_string(CustomOrbitInput[2 * INPUTRESOLUTION + 1]));

    curXpos += 30;
    glut_print_helper_medium(TEXTSTARTX, TEXTSTARTY + NEWLINE * 2, "Press e to enter and input values, press x to cancel");
}

void display_dev_info()
{
    int curYpos = 0;
    extern double VelAim;
    glut_print(TEXTSTARTX, TEXTSTARTY - curYpos, "velocity : " + to_string(velocity.abs())); curYpos += NEWLINE;
    glut_print(TEXTSTARTX, TEXTSTARTY - curYpos, "velocity aim : " + to_string(VelAim)); curYpos += NEWLINE;
}

void CustomOrbitInfo()
{
    int curYpos = TEXTSTARTY;
    glut_print_helper(TEXTSTARTX, view_height - curYpos, "Input Apogee : " + to_string(InputApogee)); curYpos += NEWLINE;
    glut_print_helper(TEXTSTARTX, view_height - curYpos, "Input Perigee : " + to_string(InputPerigee)); curYpos += NEWLINE;
    glut_print_helper(TEXTSTARTX, view_height - curYpos, "Max Altitude : " + to_string(InputApogee - LocalRadius)); curYpos += NEWLINE;
    glut_print_helper(TEXTSTARTX, view_height - curYpos, "Min Altitude : " + to_string(InputPerigee - LocalRadius)); curYpos += NEWLINE;
}

VoidFunction HelpfulInformation(COPILOT_ACTION CurrentAction)
{
    curYpos = TEXTSTARTY;
    switch (CurrentAction)
    {
        case GOTOMOON: case BIIMPULSIVEMOONTRANSFER:
            return DisplayTransferStatus;
            break;
        case SUICIDELANDING:
            return KEandEstimatedSuicideBurnWork;
        case TAKINGINPUT:
            return display_input_interface;
        case DEBUGHELPER:
            return display_dev_info;
        case CUSTOMORBIT:
            return CustomOrbitInfo;
        default:
            return NothingToShow;
    }
}