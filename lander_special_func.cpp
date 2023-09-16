#include "lander_special_func.h"

double calculateDeltaVApogee(double Apogee, double Perigee, double NewPerigee)
{
    double oldVApogee = 2 * GRAVITY * MARS_MASS * (1/Perigee - 1/Apogee) / 
                        (1 - (Perigee * Perigee) / (Apogee * Apogee));
    double newVApogee = 2 * GRAVITY * MARS_MASS * (1/NewPerigee - 1/Apogee) / 
                        (1 - (NewPerigee * NewPerigee) / (Apogee * Apogee));
    return oldVApogee - newVApogee;
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
    }
    Heights_Updated != 2;
}
