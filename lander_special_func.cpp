#include "lander_special_func.h"

double calculateNewVApogee(double Apogee, double NewPerigee)
{
    return 2 * GRAVITY * MARS_MASS * (1/NewPerigee - 1/Apogee) / 
                        (1 - (NewPerigee * NewPerigee) / (Apogee * Apogee));
}

double rocketEquationForFuel(double deltaV)
{
    // ln(m1/m2) * Vout = deltaV
    // exp(deltaV/Vout) = m1/m2
    // deltaM = fuelToBurn = m1(exp(-deltaV/Vout) - 1)
    // m1 = LANDERMASS, m2 = LANDERMASS - fuelNeeded
    // vout = impulsePerLitre / massPerLitre (impulsePerLitre == kilogramsPerLitre * metresPerSecond)
    // maxFuelForce / maxFuelBurnRate = impulsePerLitre
    // impulseNeeded / impulsePerLitre = fuelNeeded
    double Vout = MAX_THRUST / FUEL_DENSITY;
    double deltaM = LANDERMASS * (exp( -deltaV / Vout) - 1);
    return deltaM;
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

void PreventCrashLanding()
{
    Altitude = position.abs() - MARS_RADIUS;
    double vel_minus_desired_vel = 0.5 + Kh * Altitude + velocity * position.norm();
    bool TooFast = vel_minus_desired_vel < 0.0;
    double Thrust_Desired = vel_minus_desired_vel * Kp + LANDERMASS * (MARS_MASS * LANDERMASS * GRAVITY) / (position.abs2());
    throttle = (TooFast == true) ? (Thrust_Desired/MAX_THRUST):0.0;
}

void PlanDeorbitIfInPermanentOrbit()
{
    if (Heights_Updated) // This means we are in a permanent orbit as
                       // otherwise perigee and apogee wouldn't have been measured
    {
        // Put us into a landing orbit
        double NewPerigee = 40000; // 40km should do
        if (position.abs() > Greatest_Height * 0.99)
        {
            double fuelToBurn = calculateFuelBurnedForLowerPerigee(Greatest_Height, 
                                                                Lowest_Height, NewPerigee);
            Planned_Fuel_Left = fuel - fuelToBurn / FUEL_CAPACITY;
            Orbit_Change_Burn = true;//Planned_Fuel_Left > 0.0;
        }
        Orbit_Change_Burn = true;
    }
}

void AutoDeployParachuteWhenReady()
{
    if ( safe_to_deploy_parachute() && 0.5 + Kh * Altitude < MAX_PARACHUTE_SPEED && Altitude < 20000 )
    {
        parachute_status = DEPLOYED;
    }
}

extern double climb_speed;
bool climbing_done = false;
bool descending_done = false;
int done = 0;

void UpdateHeights()
{
    if (!Heights_Updated)
    {
        if (climb_speed > 0.0)
        {
            climbing_done = true;
            Greatest_Height = (position.abs() > Greatest_Height) ? position.abs() : Greatest_Height;
            if (descending_done)
            {
                descending_done = false;
                done++;
            }
        }
        if (climb_speed < 0.0)
        {
            descending_done = true;
            Lowest_Height = (position.abs() < Lowest_Height) ? position.abs() : Lowest_Height;
            if (climbing_done)
            {
                climbing_done = false;
                done++;
            }
        }
        Heights_Updated = true;
    }
}

void ClearHeights()
{
    climbing_done = false;
    descending_done = false;
    done = 0;
    Heights_Updated = false;
    Greatest_Height = 0.0;
    Lowest_Height = DBL_MAX;
}
