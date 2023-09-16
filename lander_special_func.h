#include "lander.h"
#ifndef LANDER_SPECIAL_FUNC_H
#define LANDER_SPECIAL_FUNC_H

#define FUELMASS FUEL_DENSITY * fuel
#define VELOCITYVERLET (1.0 / delta_t) * (position - positionNMinus1)
#define DRAGCONSTANT(Cd) -0.5 * atmospheric_density(position) * Cd * M_PI
#define VELCONSTANT velocity.norm() * velocity.abs2()
#define LANDERMASS (UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel)
#define Kp 0.07 * LANDERMASS
#define Kh 0.05
#define NSperLITRE (MAX_THRUST / FUEL_RATE_AT_MAX_THRUST)

double calculateDeltaV(const vector3d& position, const vector3d& velocity,
                          double altitudeThreshold);

void face_travel_direction();

bool ReachedEscapeVelocity();

#endif