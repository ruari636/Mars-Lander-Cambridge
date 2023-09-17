#include "lander.h"
#include "float.h"

#ifndef LANDER_SPECIAL_FUNC_H
#define LANDER_SPECIAL_FUNC_H

#define FUELMASS FUEL_DENSITY * fuel
#define VELOCITYVERLET (1.0 / delta_t) * (position - positionNMinus1)
#define DRAGCONSTANT(Cd) -0.5 * atmospheric_density(position) * Cd * M_PI
#define VELCONSTANT velocity.norm() * velocity.abs2()
#define LANDERMASS (UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel)
#define Kp 0.5
#define Kh 0.05
#define NSperLITRE (MAX_THRUST / FUEL_RATE_AT_MAX_THRUST)
#define LAGBURNCORRECTION 0.5

double calculateNewVApogee(double Apogee, double NewPerigee);

double rocketEquationForFuel(double deltaV); // returns how much fuel to burn

double calculateFuelBurnedForLowerPerigee(double Apogee, double Perigee, double NewPerigee);

void face_travel_direction();

bool ReachedEscapeVelocity();

void PreventLanderEscape();

void ClampVelocity(double clamp);

void PreventCrashLanding();

void PlanDeorbitIfInPermanentOrbit();

void AutoDeployParachuteWhenReady();

void UpdateHeights(); // returns false when we have gathered all info about current elliptical orbit

void ClearHeights();

bool StartSuicideBurn();

#endif