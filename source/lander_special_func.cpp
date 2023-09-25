#include "lander_special_func.h"

extern vector3d FGravMars;
extern vector3d FGravMoon;
extern vector3d FDragLander;
extern vector3d FDragChute;
extern double climb_speed;

bool HeightsUpdated = false;
double Greatest_Height = 0.0;
double Lowest_Height = DBL_MAX;
bool OrbitChangeBurn = false;
double Planned_Fuel_Left = fuel;
bool SuicideBurnStarted = false;
double VelDescent;
bool previous_descending;
bool descending;
double avgLanderMassInBurn;
double DragEstimate;
double ForceEstimate;
double KE;
double EstimatedTimeToBurnSuicide;

void InitialiseSpecialFunc()
{
    ClearHeights();
    done = 0;
    SuicideBurnStarted = false;
    OrbitChangeBurn = false;
    HeightsUpdated = false;
    previous_descending = false;
    descending = false; 
    VelDescent = 0.5;
    Planned_Fuel_Left = fuel;
    avgLanderMassInBurn = LANDERMASS;
    throttle = 0.0;
}

void ResetAutoPilot()
{
    InitialiseSpecialFunc();
}

void FaceDirection(vector3d dir)
{
  vector3d up, left, out;
  double m[16];

  up = dir.norm(); // this is the direction we want the lander's nose to point in

  // !!!!!!!!!!!!! HINT TO STUDENTS ATTEMPTING THE EXTENSION EXERCISES !!!!!!!!!!!!!!
  // For any-angle attitude control, we just need to set "up" to something different,
  // and leave the remainder of this function unchanged. For example, suppose we want
  // the attitude to be stabilized at stabilized_attitude_angle to the vertical in the
  // close-up view. So we need to rotate "up" by stabilized_attitude_angle degrees around
  // an axis perpendicular to the plane of the close-up view. This axis is given by the
  // vector product of "up"and "closeup_coords.right". To calculate the result of the
  // rotation, search the internet for information on the axis-angle rotation formula.

  // Set left to something perpendicular to up
  left.x = -up.y; left.y = up.x; left.z = 0.0;
  if (left.abs() < SMALL_NUM) {left.x = -up.z; left.y = 0.0; left.z = up.x;}
  left = left.norm();  
  out = left^up;
  // Construct modelling matrix (rotation only) from these three vectors
  m[0] = out.x; m[1] = out.y; m[2] = out.z; m[3] = 0.0;
  m[4] = left.x; m[5] = left.y; m[6] = left.z; m[7] = 0.0;
  m[8] = up.x; m[9] = up.y; m[10] = up.z; m[11] = 0.0;
  m[12] = 0.0; m[13] = 0.0; m[14] = 0.0; m[15] = 1.0;
  // Decomponse into xyz Euler angles
  orientation = matrix_to_xyz_euler(m);
}

vector3d VecAtAngleToPosInPlane(double angle)
{
    vector3d PlaneNormal = MarsSphereOfInfluence ? position.crossProduct(velocity).norm():(-MoonRelPos).crossProduct(OrbitVel).norm();
    vector3d PosNorm = MarsSphereOfInfluence ? position.norm():(MoonRelPos).norm();
    if (velocity.abs2() == 0 || PlaneNormal.abs2() == 0) return PosNorm;
    vector3d PosPerp = PosNorm.crossProduct(PlaneNormal);
    double xFac = sin(angle);
    double yFac = -cos(angle);
    return PosPerp * xFac + PosNorm * yFac;
}

bool ReachedEscapeVelocity()
{
    return velocity.abs2() > 2.0 * GRAVITY * MARS_MASS / position.abs();
}

void PreventLanderEscape()
{
    if (ReachedEscapeVelocity())
    {
        // throttle = 1.0;
    }
}

void ClampVelocity(double clamp) // assuming down is positive
{
    if (velocity * position <= 0.0) throttle = (velocity.abs() > clamp) ? 1.0:0.0;
}

#define HEIGHTTOLANDINGSPEED 216.0
void ThrustProportionalToUnsafeVel()
{
    double velMinusDesiredVel;
    if (Altitude > HEIGHTTOLANDINGSPEED) 
    { 
        velMinusDesiredVel = VelDescent + (MarsSphereOfInfluence ? Kh:0.01) * Altitude + 
                             OrbitVel * (MarsSphereOfInfluence ? position.norm():(-MoonRelPos.norm()));
    }
    else if (MarsSphereOfInfluence) velMinusDesiredVel = 0.5 + Kh * Altitude + OrbitVel * (position.norm());
    else velMinusDesiredVel = 0.5 + 0.01 * Altitude - OrbitVel * MoonRelPos.norm();
    bool TooFast = velMinusDesiredVel < 0.0;      // if moon gravity is on this can actually cause our lander to crash into mars
    double Thrust_Desired = -velMinusDesiredVel * (MoonGravityEnabled ? Kp:0.66) + (MostImportantMass * LANDERMASS * GRAVITY) / 
                            (pow(Altitude + LocalRadius, 2) * MAX_THRUST);
    throttle = (TooFast == true) ? min((Thrust_Desired), 1.0):0.0;
}

void AutoDeployParachuteWhenReady()
{
    if ( safe_to_deploy_parachute() && 0.5 + Kh * MarsAltitude < MAX_PARACHUTE_SPEED && MarsAltitude < 20000 )
    {
        parachute_status = DEPLOYED;
    }
}

bool DistancesWithinError(double x0, double x1)
{
    double Error = 1.01;
    double OrbitTime = 0.0;
    if (HeightsUpdated) { OrbitTime = KeplerPeriod((Greatest_Height + Lowest_Height) / 2.0); }
    if (MoonGravityEnabled)
    {
        if (MarsSphereOfInfluence)
        {
            Error += (FGravMoon.abs() * OrbitTime * OrbitTime) / (x0 + x1);
        }
        else
        {
            Error += (FGravMars.abs() * OrbitTime * OrbitTime) / (x0 + x1);
        }
    }
    return x1 < x0 * Error && x1 > x0 / Error;
}

void UpdateHeights()
{
   previous_descending = descending;
   descending = signbit(climb_speed);
   if (!descending && previous_descending) // we have just gone past the lowest point in the orbit, measure it
                                                                   // also due to orbital mechanics, when deorbiting the altitude can increase for a bit so an extra check was added
   {
        done |= LOWESTHEIGHTMEASUREDMASK;
        if ((done & MOONAPROACHBURNFINISHED) == MOONAPROACHBURNFINISHED) done |= HYPERBOLICPERIGEEMASK;
        if (!DistancesWithinError(Lowest_Height, Altitude + LocalRadius))
        {
            HeightsUpdated = false;
            done &= !LOWESTHEIGHTMEASUREDMASK;
        }
        Lowest_Height = Altitude + LocalRadius;
   }
   else if (descending && !previous_descending)                    // we have just gone past the highest point in the orbit, measure it
   {
        done |= GREATESTHEIGHTMEASUREDMASK;
        if (!DistancesWithinError(Greatest_Height, Altitude + LocalRadius))
        {
            HeightsUpdated = false;
            done &= !GREATESTHEIGHTMEASUREDMASK;
        }
        Greatest_Height = Altitude + LocalRadius;
   }
   if (done & (LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK) == (LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK))
   {
        HeightsUpdated = true;
   }
}

bool PerigeeDescending;
bool PreviousPerigeeDescending;
void ReachedHyperbolicPerigee()
{
   previous_descending = PerigeeDescending;
   PerigeeDescending = signbit(climb_speed);
   if (!PerigeeDescending && PreviousPerigeeDescending)
   {
    done |= HYPERBOLICPERIGEEMASK;
   }
}

void ClearHeights()
{
    done &= !(LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK);
    HeightsUpdated = false;
    Greatest_Height = 0.0;
    Lowest_Height = DBL_MAX;
}

void IterativeSuicideBurnEstimator()
{
    for (int i = 0; i < 5; i++)
    {
        ForceEstimate = MAX_THRUST - (MostImportantMass * avgLanderMassInBurn * GRAVITY) / (LocalRadius * LocalRadius);
        DragEstimate = (FDragLander.abs() + FDragChute.abs()) * (0.01) + (climb_speed * (6.0E-1)); //+ climb_speed * 5.5E-8)));// + climb_speed * 1.0E-8))); //(FDragLander.abs() + FDragChute.abs()) * 0.01;
        //double climb_speed = velocity * position.norm();
        EstimatedTimeToBurnSuicide = velocity.abs() / 
            ((ForceEstimate + DragEstimate) / avgLanderMassInBurn); // t = -u/a when v = 0
        avgLanderMassInBurn = LANDERMASS - 
                (EstimatedTimeToBurnSuicide * FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY) / 1.5;
    }
}

void IterativeSuicideBurnEstimatorMoon()
{
    double VelVert = OrbitVel * MoonRelPos / (Altitude + MOONRADIUS);
    for (int i = 0; i < 5; i++)
    {
        ForceEstimate = MAX_THRUST - (MostImportantMass * avgLanderMassInBurn * GRAVITY) / (LocalRadius * LocalRadius);
        EstimatedTimeToBurnSuicide = abs(VelVert) / 
            (ForceEstimate / avgLanderMassInBurn); // t = -u/a when v = 0
        avgLanderMassInBurn = LANDERMASS - 
                (EstimatedTimeToBurnSuicide * FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY) / 1.6;
    }
}

bool UpdateSuicideBurn()
{
    if (MarsSphereOfInfluence)
    {
        if (velocity * position < -0.1)
        {
            double KEMax = (MAX_THRUST - (MostImportantMass * LANDERMASS * GRAVITY) / (LocalRadius * LocalRadius)) * Altitude; // Based on the work the lander can do against falling, neglecting drag and fuel usage reducing mass
            double VMax = sqrt(2 * KEMax / LANDERMASS); // If the velocity is below this, we must be able to start a suicide burn and stop, as drag is neglected in the calculation of KE_MAX
            if (!SuicideBurnStarted)
            {
                avgLanderMassInBurn = LANDERMASS;
                IterativeSuicideBurnEstimator();
                EnergyToBurn = (ForceEstimate + DragEstimate) * Altitude;
                bool AltitudeChecked = MarsAltitude < MAXSUICIDEBURNCHECKHEIGHT; // don't check altitude if we aren't on mars as drag won't slow us
                SuicideBurnStarted = (EnergyToBurn < KE && AltitudeChecked); // At 5km the max vel we can stop is
            }
            else if (velocity.abs() < VMax && Altitude > 5000.0) SuicideBurnStarted = false; // We entered the atmosphere so fast that suicide burn was triggered early
        }
        if (SuicideBurnStarted) VelDescent = max(sqrt(2 * ((MAX_THRUST - (MARS_MASS * LANDERMASS * GRAVITY) / (MARS_RADIUS * MARS_RADIUS)) * HEIGHTTOLANDINGSPEED) / LANDERMASS) * 0.8, 0.5); 
        return SuicideBurnStarted;
    }
    else
    {
        double MoonEnergyGrav = LANDERMASS * MOONMASS * GRAVITY / MoonAltitude * (1 / MOONRADIUS - 1 / (Altitude + MOONRADIUS));
        if (!SuicideBurnStarted)
        { IterativeSuicideBurnEstimatorMoon(); }
        double VelVert = OrbitVel * MoonRelPos / (Altitude + MOONRADIUS);
        double KE_Vertical = 0.5 * avgLanderMassInBurn * VelVert * VelVert;
        double KE_Horizontal = KE - KE_Vertical;
        SuicideBurnStarted = KE_Vertical + 0.67 * KE_Horizontal + MoonEnergyGrav > ForceEstimate * MoonAltitude | SuicideBurnStarted;
        if (SuicideBurnStarted) 
        {
            VelDescent = max(sqrt(2 * ((MAX_THRUST - (MOONMASS * LANDERMASS * GRAVITY) / (MOONRADIUS * MOONRADIUS)) * HEIGHTTOLANDINGSPEED) / LANDERMASS) * 0.8, 0.5); 
            if (Altitude < HEIGHTTOLANDINGSPEED)
            VelDescent = 0.5;
        }
        return SuicideBurnStarted;
    }
}

void LandSuicide()
{    
    KE = 0.5 * LANDERMASS * velocity.abs2();
    if (!OrbitChangeBurn)
    {
        if (SuicideBurnStarted)
        {
            AutoDeployParachuteWhenReady();
            ThrustProportionalToUnsafeVel();
        }
        UpdateSuicideBurn();
    }
}

void LandProportional()
{
    AutoDeployParachuteWhenReady();
    if (!OrbitChangeBurn) ThrustProportionalToUnsafeVel();
}