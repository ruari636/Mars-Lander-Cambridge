#include "lander_special_func.h"

#ifndef ORBIT_TRANSFER_H
#define ORBIT_TRANSFER_H

#define ORBITCHANGECALCDONE 0x0004
#define NEXTAPOGEEMET 0x0008
#define NEXTPERIGEEMET 0x0010
#define MOONAPROACHBURNFINISHED 0x0020
#define MOONESCAPEPREVENTED 0x0040

#define MOONAPROACHAPOGEE (MoonDistance + MOONRADIUS + EXOSPHERE)

extern bool MoonApproachStarted;

void InitialiseOrbitTransfers();

void OrbitChangeBurner();

double calculateNewVApogee(double Apogee, double NewPerigee);

double calculateNewVPerigee(double Perigee, double NewApogee);

double rocketEquationForFuel(double deltaV); // returns how much fuel to burn

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee);

double calculateFuelBurnedForNewApogee(double Apogee, double Perigee, double NewApogee);

void PlanDeorbitIfInPermanentOrbit();

void Deorbit();

void CirculariseCurrentOrbit();

void MoveToOrbitInPlane(double NextApogee, double NextPerigee);

void ApproachMoon();

void PreventMoonEscape();

#endif