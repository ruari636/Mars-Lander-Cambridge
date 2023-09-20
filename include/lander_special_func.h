#ifndef LANDER_SPECIAL_FUNC_H
#define LANDER_SPECIAL_FUNC_H

#include "all.h"
#include "float.h"

#define FUELMASS FUEL_DENSITY * fuel
#define VELOCITYVERLET (1.0 / delta_t) * (position - positionNMinus1)
#define DRAGCONSTANT(Cd) -0.5 * atmospheric_density(position) * Cd * M_PI
#define VELCONSTANT velocity.norm() * velocity.abs2()
#define LANDERMASS (UNLOADED_LANDER_MASS + FUEL_DENSITY * fuel)
#define Kp 0.5
#define Kh 0.05
#define NSperLITRE (MAX_THRUST / FUEL_RATE_AT_MAX_THRUST)
#define MAXSUICIDEBURNCHECKHEIGHT 7500

#define LOWESTHEIGHTMEASUREDMASK 0x0001
#define GREATESTHEIGHTMEASUREDMASK 0x0002

void InitialiseSpecialFunc();

void FaceDirection(vector3d dir);

vector3d VecAtAngleToPosInPlane(double angle);

bool ReachedEscapeVelocity();

void PreventLanderEscape();

void ClampVelocity(double clamp);

void ThrustProportionalToUnsafeVel();

void AutoDeployParachuteWhenReady();

void UpdateHeights(); // returns false when we have gathered all info about current elliptical orbit

void ClearHeights();

bool UpdateSuicideBurn();

void LandSuicide();

void LandProportional();

#endif