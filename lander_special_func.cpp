#include "lander_special_func.h"
#include "lander.h"

extern vector3d FGrav;
extern vector3d FDragLander;
extern vector3d FDragChute;
extern double climb_speed;

bool Heights_Updated = false;
double Greatest_Height = 0.0;
double Lowest_Height = DBL_MAX;
bool Orbit_Change_Burn = false;
double Planned_Fuel_Left = fuel;
uint16_t done = 0; // variable containing ored together event flags
bool SuicideBurnStarted = false;
double VelDescent;
bool previous_descending;
bool descending;
double avgLanderMassInBurn;
double DragEstimate;
double ForceEstimate;

void initialize_special_func()
{
    ClearHeights();
    done = 0;
    SuicideBurnStarted = false;
    Orbit_Change_Burn = false;
    Heights_Updated = false;
    previous_descending = false;
    descending = false; 
    VelDescent = 0.5;
    Planned_Fuel_Left = fuel;
    avgLanderMassInBurn = LANDERMASS;
}

void OrbitChangeBurner()
{
    if (Orbit_Change_Burn)
    {
        if (Planned_Fuel_Left < fuel)
        {
            throttle = 1.0;
        }
        else
        {
            Orbit_Change_Burn = false;
            ClearHeights(); // New orbit, new heights
            throttle = 0.0;
        }
    }
}

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

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee)
{
    double v1 = calculateNewVApogee(Apogee, Perigee);
    double v2 = calculateNewVApogee(Apogee, NewPerigee);
    return rocketEquationForFuel(v2 - v1);
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
    //if (velocity * position <= 0.0) throttle = (velocity.abs() > clamp) ? 1.0:0.0;
}

#define HEIGHTTOLANDINGSPEED 216.0
void ThrustProportionalToUnsafeVel()
{
    double velMinusDesiredVel;
    if (altitude > HEIGHTTOLANDINGSPEED) { velMinusDesiredVel = VelDescent + Kh * altitude + velocity * position.norm(); }
    else velMinusDesiredVel = 0.5 + Kh * altitude + velocity * position.norm();
    bool TooFast = velMinusDesiredVel < 0.0;
    double Thrust_Desired = -velMinusDesiredVel * Kp + (MARS_MASS * LANDERMASS * GRAVITY) / (position.abs2() * MAX_THRUST);
    throttle = (TooFast == true) ? min((Thrust_Desired), 1.0):0.0;
}

void PlanDeorbitIfInPermanentOrbit()
{
    if (Heights_Updated) // This means we are in a permanent orbit as
                       // otherwise perigee and apogee wouldn't have been measured
    {
        // Put us into a landing orbit
        double NewPerigee = EXOSPHERE * 0.6 + MARS_RADIUS; // To use drag to slow us down
        if (position.abs() > Greatest_Height * 0.99)
        {
            double fuelToBurn = calculateFuelBurnedForNewPerigee(Greatest_Height, 
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

void UpdateHeights()
{
   previous_descending = descending;
   descending = signbit(climb_speed);
   if (!descending && previous_descending) // we have just gone past the lowest point in the orbit, measure it
   {
        if (Lowest_Height < position.abs() * 1.01) done |= LOWESTHEIGHTMEASUREDMASK;// We have gone past the lowest point twice and it is the same so we are in a permanent orbit
        Lowest_Height = position.abs();
   }
   else if (descending && !previous_descending) // we have just gone past the highest point in the orbit, measure it
   {
        Greatest_Height = position.abs();
        done |= GREATESTHEIGHTMEASUREDMASK;
   }
   if (done & (LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK) == (LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK))
   {
        Heights_Updated = true;
        done &= !(LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK); // reset the flags that correspond to UpdateHeights
   }
}

void ClearHeights()
{
    done &= !(LOWESTHEIGHTMEASUREDMASK + GREATESTHEIGHTMEASUREDMASK);
    Heights_Updated = false;
    Greatest_Height = 0.0;
    Lowest_Height = DBL_MAX;
}

void IterativeSuicideBurnEstimator()
{
    for (int i = 0; i < 5; i++)
    {
        ForceEstimate = MAX_THRUST - (MARS_MASS * avgLanderMassInBurn * GRAVITY) / (MARS_RADIUS * MARS_RADIUS);
        DragEstimate = (FDragLander.abs() + FDragChute.abs()) * (0.008);// + (VelTowardsGnd * (5.4E-6 + VelTowardsGnd * 5.5E-8)));// + VelTowardsGnd * 1.0E-8))); //(FDragLander.abs() + FDragChute.abs()) * 0.01;
        //double VelTowardsGnd = velocity * position.norm();
        double EstimatedTimeToBurn = velocity.abs() / 
            ((ForceEstimate + DragEstimate) / avgLanderMassInBurn); // t = -u/a when v = 0
        avgLanderMassInBurn = LANDERMASS - 
                (EstimatedTimeToBurn * FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY) / 1.5;
    }
}

bool UpdateSuicideBurn()
{
    if (velocity * position < -0.1)
    {
        double KEMax = (MAX_THRUST - (MARS_MASS * LANDERMASS * GRAVITY) / (MARS_RADIUS * MARS_RADIUS)) * altitude; // Based on the work the lander can do against falling, neglecting drag and fuel usage reducing mass
        double VMax = sqrt(2 * KEMax / LANDERMASS); // If the velocity is below this, we must be able to start a suicide burn and stop, as drag is neglected in the calculation of KE_MAX
        if (!SuicideBurnStarted)
        {
            avgLanderMassInBurn = LANDERMASS;
            double KE = 0.5 * LANDERMASS * velocity.abs2();
            IterativeSuicideBurnEstimator();
            SuicideBurnStarted = (((ForceEstimate + DragEstimate) * altitude < KE) && altitude < MAXSUICIDEBURNCHECKHEIGHT); // At 5km the max vel we can stop is
        }
        else if (velocity.abs() < VMax && altitude > 5000.0) SuicideBurnStarted = false; // We entered the atmosphere so fast that suicide burn was triggered early
    }
    if (SuicideBurnStarted) VelDescent = max(sqrt(2 * ((MAX_THRUST - (MARS_MASS * LANDERMASS * GRAVITY) / (MARS_RADIUS * MARS_RADIUS)) * HEIGHTTOLANDINGSPEED) / LANDERMASS) * 0.8, 0.5); 
    return SuicideBurnStarted;
}

void LandSuicide()
{
    Deorbit();
    
    if (!Orbit_Change_Burn)
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
    Deorbit();
    AutoDeployParachuteWhenReady();
    if (!Orbit_Change_Burn) ThrustProportionalToUnsafeVel();
}

void Deorbit()
{
    PlanDeorbitIfInPermanentOrbit(); // Sets Orbit_Change_Burn to true if a deorbit
                                    // is possible with remaining fuel
    OrbitChangeBurner();
}

void CirculariseCurrentOrbit()
{
    if (Heights_Updated) // We have collected current data on Apogee and Perigee
    {
        if (position.abs() > Greatest_Height * 0.999) // Only run the burner when we are very close to apogee
        {
            if ((done & CIRCULARISEORBITCALCDONE) == 0)
            {
                double fuelToBurn = (-calculateFuelBurnedForNewPerigee(Greatest_Height, // set to negative as we are burning in direction of travel
                                                                Lowest_Height, Greatest_Height)) > 0.0 ? -calculateFuelBurnedForNewPerigee(Greatest_Height, Lowest_Height, Greatest_Height):0.0;
                Planned_Fuel_Left = fuel - (fuelToBurn / (FUEL_CAPACITY * FUEL_DENSITY));
                done |= CIRCULARISEORBITCALCDONE;
            }
            Orbit_Change_Burn = Planned_Fuel_Left > 0.0;
            OrbitChangeBurner();
        }
    }
    if (Planned_Fuel_Left <= 0.0)
    {
        ClearHeights();
        done &= !CIRCULARISEORBITCALCDONE;
    }
}
