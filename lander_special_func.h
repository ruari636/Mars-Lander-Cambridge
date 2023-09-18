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
#define MAXSUICIDEBURNCHECKHEIGHT 7500

#define LOWESTHEIGHTMEASUREDMASK 0x0001
#define GREATESTHEIGHTMEASUREDMASK 0x0002
#define ORBITCHANGECALCDONE 0x0004
#define NEXTAPOGEEMET 0x0008
#define NEXTPERIGEEMET 0x0010

void initialize_special_func();

void OrbitChangeBurner();

double calculateNewVApogee(double Apogee, double NewPerigee);

double calculateNewVPerigee(double Perigee, double NewApogee);

double rocketEquationForFuel(double deltaV); // returns how much fuel to burn

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee);

double calculateFuelBurnedForNewApogee(double Apogee, double Perigee, double NewApogee);

void FaceDirection(vector3d dir);

vector3d VecAtAngleToPosInPlane(double angle);

bool ReachedEscapeVelocity();

void PreventLanderEscape();

void ClampVelocity(double clamp);

void ThrustProportionalToUnsafeVel();

void PlanDeorbitIfInPermanentOrbit();

void AutoDeployParachuteWhenReady();

void UpdateHeights(); // returns false when we have gathered all info about current elliptical orbit

void ClearHeights();

bool UpdateSuicideBurn();

void LandSuicide();

void LandProportional();

void Deorbit();

void CirculariseCurrentOrbit();

void MoveToOrbitInPlane(double NextApogee, double NextPerigee);

#endif