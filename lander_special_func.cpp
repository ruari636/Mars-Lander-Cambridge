#include "lander_special_func.h"

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

// Function to calculate the impulse required for the maneuver
double calculateDeltaV(const vector3d& position, const vector3d& velocity, double altitudeThreshold, double fuel) {
    // Constants
    const double FUEL_FORCE = NSperLITRE; // Force produced by burning one unit of fuel (adjust as needed)

    // Calculate the current mass of the lander
    double landerMass = UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel;

    // Calculate the specific orbital energy
    double r = position.abs();
    double v = velocity.abs();
    double specificOrbitalEnergy = 0.5 * v * v - GRAVITY * MARS_MASS / r;

    // Calculate the velocity needed at the highest point to achieve the desired altitude threshold
    double rThreshold = r - altitudeThreshold;
    double vThreshold = sqrt(2 * GRAVITY * MARS_MASS / rThreshold);

    // Calculate the required velocity change (impulse) at the highest point
    double deltaV = vThreshold - v;

    // Calculate the required impulse to account for fuel consumption
    double deltaV_fuel = (landerMass / (landerMass - FUEL_DENSITY)) * (exp(FUEL_FORCE / (landerMass - FUEL_DENSITY)) - 1);

    // Combine the two impulses
    deltaV = deltaV + deltaV_fuel; // Assuming fuel is burned along the x-axis

    return deltaV;
}