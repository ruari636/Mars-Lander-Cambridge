#include "lander_special_func.h"
#include "lander.h"

bool Heights_Updated = false;
double Greatest_Height = 0.0;
double Lowest_Height = DBL_MAX;
bool Orbit_Change_Burn = false;
bool debugBurner = false;
double Planned_Fuel_Left;
uint16_t done = 0; // variable containing ored together event flags
double Time_Burn;
double Last_Throttle;
bool SlowedDown = false;

double calculateNewVApogee(double Apogee, double NewPerigee)
{
    return sqrt(GRAVITY * MARS_MASS * (2 * NewPerigee) / 
                        (Apogee * (NewPerigee + Apogee)));
}

double rocketEquationForFuel(double deltaV)
{
    // ln(m1/m2) * Vout = deltaV
    // exp(deltaV/Vout) = m1/m2
    // deltaM = fuelToBurn = m1(exp(-deltaV/Vout) - 1)
    // m1 = LANDERMASS, m2 = LANDERMASS - fuelNeeded
    // maxFuelForce / maxFuelBurnRate = impulsePerLitre
    // impulseNeeded / impulsePerLitre = fuelNeeded
    // thrust = dm/dt * Vout, Vout = maxThrust / fuelRateAtMaxThrust (kg/s)
    double Vout = MAX_THRUST / (FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY);
    double deltaM = LANDERMASS / exp(deltaV/Vout) - LANDERMASS;
    return deltaM; // We want to know how much fuel to burn
}

double calculateFuelBurnedForLowerPerigee(double Apogee, double Perigee, double NewPerigee)
{
    double v1 = calculateNewVApogee(Apogee, Perigee);
    double v2 = calculateNewVApogee(Apogee, NewPerigee);
    return rocketEquationForFuel(v2 - v1);
}

void face_travel_direction()
{
  vector3d up, left, out;
  double m[16];

  up = -velocity.norm(); // this is the direction we want the lander's nose to point in

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

bool ReachedEscapeVelocity()
{
    return 0.5 * velocity.abs2() > GRAVITY * MARS_MASS / position.abs();
}

void PreventLanderEscape()
{
    if (ReachedEscapeVelocity())
    {
        throttle = 1.0;
    }
}

void ClampVelocity(double clamp) // assuming down is positive
{
    //if (velocity * position <= 0.0) throttle = (velocity.abs() > clamp) ? 1.0:0.0;
}

void PreventCrashLanding()
{
    double vel_minus_desired_vel = 0.5 + Kh * altitude + velocity * position.norm();
    bool TooFast = vel_minus_desired_vel < 0.0;
    double Thrust_Desired = -vel_minus_desired_vel * Kp + (MARS_MASS * LANDERMASS * GRAVITY) / (position.abs2() * MAX_THRUST);
    throttle = (TooFast == true) ? min((Thrust_Desired), 1.0):0.0;
}

void PlanDeorbitIfInPermanentOrbit()
{
    if (Heights_Updated) // This means we are in a permanent orbit as
                       // otherwise perigee and apogee wouldn't have been measured
    {
        // Put us into a landing orbit
        double NewPerigee = 100000 + MARS_RADIUS; // 100km should do
        if (position.abs() > Greatest_Height * 0.98)
        {
            double fuelToBurn = calculateFuelBurnedForLowerPerigee(Greatest_Height, 
                                                                Lowest_Height, NewPerigee);
            Planned_Fuel_Left = fuel - (fuelToBurn / (FUEL_CAPACITY * FUEL_DENSITY));
            Orbit_Change_Burn = Planned_Fuel_Left > 0.0;
            ClearHeights();
        }
    }
}

void AutoDeployParachuteWhenReady()
{
    if ( safe_to_deploy_parachute() && 0.5 + Kh * altitude < MAX_PARACHUTE_SPEED && altitude < 20000 )
    {
        parachute_status = DEPLOYED;
    }
}

extern double climb_speed;
bool previous_descending;
bool descending;

void UpdateHeights()
{
   previous_descending = descending;
   descending = signbit(climb_speed);
   if (!descending && previous_descending) // we have just gone past the lowest point in the orbit, measure it
   {
    Lowest_Height = position.abs();
    done |= 0x0001;
   }
   else if (descending && !previous_descending) // we have just gone past the highest point in the orbit, measure it
   {
    Greatest_Height = position.abs();
    done |= 0x0002;
   }
   if (done & (0x0001 + 0x0002) == (0x0001 + 0x0002))
   {
    Heights_Updated = true;
    done &= !(0x0001 + 0x0002);
   }
}

void ClearHeights()
{
    done = 0;
    Heights_Updated = false;
    Greatest_Height = 0.0;
    Lowest_Height = DBL_MAX;
}

extern vector3d FGrav;
extern vector3d FDragLander;
extern vector3d FDragChute;
bool SuicideBurnStarted = false;

double avgLanderMassInBurn;
double ForceEstimate;
double DragEstimate;
double EstimatedTimeToBurn;

void IterativeSuicideBurnEstimator()
{
    for (int i = 0; i < 5; i++)
    {
        ForceEstimate = MAX_THRUST - (MARS_MASS * avgLanderMassInBurn * GRAVITY) / (MARS_RADIUS * MARS_RADIUS);
        DragEstimate = (FDragLander.abs() + FDragChute.abs()) * 0.01;
        EstimatedTimeToBurn = velocity.abs() / 
            ((ForceEstimate + DragEstimate) / avgLanderMassInBurn); // t = -u/a when v = 0
        avgLanderMassInBurn = LANDERMASS - 
                (EstimatedTimeToBurn * FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY) / 1.5;
    }
}

bool StartSuicideBurn()
{
    if (velocity * position < 0.0)
    {
        avgLanderMassInBurn = LANDERMASS;
        double KE = 0.5 * LANDERMASS * velocity.abs2();
        IterativeSuicideBurnEstimator();
        SuicideBurnStarted = ((ForceEstimate + DragEstimate) * altitude < KE) && altitude < 15000 | SuicideBurnStarted;
    }
    return SuicideBurnStarted;
}

void LandSuicide()
{
    Deorbit();
    
    if (!Orbit_Change_Burn)
    {
        if (SlowedDown)
        {
            PreventCrashLanding();
        }
        else if (StartSuicideBurn())
        {
            SlowedDown = true;
        }
        AutoDeployParachuteWhenReady();
    }
}

void LandProportional()
{
    Deorbit();
    AutoDeployParachuteWhenReady();
    if (!Orbit_Change_Burn) PreventCrashLanding();
}

void Deorbit()
{
    PlanDeorbitIfInPermanentOrbit(); // Sets Orbit_Change_Burn to true if a deorbit
                                    // is possible with remaining fuel
    if (Orbit_Change_Burn)
    {
        if (Planned_Fuel_Left <= fuel)// && Planned_Fuel_Left > 0.0)
        {
            throttle = 1.0;
        }
        else
        {
            Orbit_Change_Burn = false;
            throttle = 0.0;
        }
    }
}
