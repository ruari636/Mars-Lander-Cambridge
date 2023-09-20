#include "lander_special_func.h"

extern vector3d FGravMars;
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
}

void FaceDirection(vector3d dir)
{
  vector3d up, left, out;
  double m[16];

  up = -dir.norm(); // this is the direction we want the lander's nose to point in

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
    vector3d PlaneNormal = position.crossProduct(velocity).norm();
    vector3d PosNorm = position.norm();
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
    if (MarsAltitude > HEIGHTTOLANDINGSPEED) { velMinusDesiredVel = VelDescent + Kh * MarsAltitude + velocity * position.norm(); }
    else velMinusDesiredVel = 0.5 + Kh * MarsAltitude + velocity * position.norm();
    bool TooFast = velMinusDesiredVel < 0.0;
    double Thrust_Desired = -velMinusDesiredVel * Kp + (MARS_MASS * LANDERMASS * GRAVITY) / (position.abs2() * MAX_THRUST);
    throttle = (TooFast == true) ? min((Thrust_Desired), 1.0):0.0;
}

void AutoDeployParachuteWhenReady()
{
    if ( safe_to_deploy_parachute() && 0.5 + Kh * MarsAltitude < MAX_PARACHUTE_SPEED && MarsAltitude < 20000 )
    {
        parachute_status = DEPLOYED;
    }
}

bool WithinError(double x0, double x1)
{
    return x1 < x0 * 1.01 && x1 > x0 * 0.99;
}

void UpdateHeights()
{
   previous_descending = descending;
   descending = signbit(climb_speed);
   if (!descending && previous_descending) // we have just gone past the lowest point in the orbit, measure it
   {
        Lowest_Height = position.abs();
        done |= LOWESTHEIGHTMEASUREDMASK;
        if (!WithinError(Lowest_Height, position.abs()))
        {
            HeightsUpdated = false;
            done &= !LOWESTHEIGHTMEASUREDMASK;
        }
   }
   else if (descending && !previous_descending) // we have just gone past the highest point in the orbit, measure it
   {
        Greatest_Height = position.abs();
        done |= GREATESTHEIGHTMEASUREDMASK;
        if (!WithinError(Greatest_Height, position.abs()))
        {
            HeightsUpdated = false;
            done &= !GREATESTHEIGHTMEASUREDMASK;
        }
   }
   if (done & (LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK) == (LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK))
   {
        HeightsUpdated = true;
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
        ForceEstimate = MAX_THRUST - (MARS_MASS * avgLanderMassInBurn * GRAVITY) / (MARS_RADIUS * MARS_RADIUS);
        DragEstimate = (FDragLander.abs() + FDragChute.abs()) * (0.01);// + (VelTowardsGnd * (5.4E-6 + VelTowardsGnd * 5.5E-8)));// + VelTowardsGnd * 1.0E-8))); //(FDragLander.abs() + FDragChute.abs()) * 0.01;
        //double VelTowardsGnd = velocity * position.norm();
        EstimatedTimeToBurnSuicide = velocity.abs() / 
            ((ForceEstimate + DragEstimate) / avgLanderMassInBurn); // t = -u/a when v = 0
        avgLanderMassInBurn = LANDERMASS - 
                (EstimatedTimeToBurnSuicide * FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY) / 1.5;
    }
}

bool UpdateSuicideBurn()
{
    if (velocity * position < -0.1)
    {
        double KEMax = (MAX_THRUST - (MARS_MASS * LANDERMASS * GRAVITY) / (MARS_RADIUS * MARS_RADIUS)) * MarsAltitude; // Based on the work the lander can do against falling, neglecting drag and fuel usage reducing mass
        double VMax = sqrt(2 * KEMax / LANDERMASS); // If the velocity is below this, we must be able to start a suicide burn and stop, as drag is neglected in the calculation of KE_MAX
        if (!SuicideBurnStarted)
        {
            avgLanderMassInBurn = LANDERMASS;
            IterativeSuicideBurnEstimator();
            EnergyToBurn = (ForceEstimate + DragEstimate) * Altitude;
            SuicideBurnStarted = (EnergyToBurn < KE && MarsAltitude < MAXSUICIDEBURNCHECKHEIGHT); // At 5km the max vel we can stop is
        }
        else if (velocity.abs() < VMax && Altitude > 5000.0) SuicideBurnStarted = false; // We entered the atmosphere so fast that suicide burn was triggered early
    }
    if (SuicideBurnStarted) VelDescent = max(sqrt(2 * ((MAX_THRUST - (MARS_MASS * LANDERMASS * GRAVITY) / (MARS_RADIUS * MARS_RADIUS)) * HEIGHTTOLANDINGSPEED) / LANDERMASS) * 0.8, 0.5); 
    return SuicideBurnStarted;
}

void LandSuicide()
{    
    KE = 0.5 * LANDERMASS * velocity.abs2();
    if (!OrbitChangeBurn)
    {
        if (SuicideBurnStarted)
        {
            ThrustProportionalToUnsafeVel();
        }
        UpdateSuicideBurn();
        AutoDeployParachuteWhenReady();
    }
}

void LandProportional()
{
    AutoDeployParachuteWhenReady();
    if (!OrbitChangeBurn) ThrustProportionalToUnsafeVel();
}