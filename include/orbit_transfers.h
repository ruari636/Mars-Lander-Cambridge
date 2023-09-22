#include "lander_special_func.h"

#ifndef ORBIT_TRANSFER_H
#define ORBIT_TRANSFER_H

#define ORBITCHANGECALCDONE 0x0004
#define NEXTAPOGEEMET 0x0008
#define NEXTPERIGEEMET 0x0010
#define MOONAPROACHBURNFINISHED 0x0020
#define MOONESCAPEPREVENTED 0x0040

extern bool MoonApproachStarted;

void InitialiseOrbitTransfers();

void OrbitChangeBurner();

void OrbitChangeBurnerVel();

void OrbitChangeBurnerVel(vector3d dir);

double calculateNewV(double Apogee, double Perigee, double CurDist);

double rocketEquationForFuel(double v2, double v1);

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee);

double calculateFuelBurnedForNewApogee(double Apogee, double Perigee, double NewApogee);

void ChangePerigee(double NextPerigee);

void ChangeApogee(double NextApogee);

void PlanDeorbitIfInPermanentOrbit();

void Deorbit();

void CirculariseCurrentOrbit();

void MoveToOrbitInPlane(double NextApogee, double NextPerigee);

double CalculateAngleXY(const vector3d& A, const vector3d& B, const vector3d& C);

double KeplerPeriod(double SemiMajorAxis);

void ApproachMoon();

double HyperbolicPerigee();

bool PreventMoonEscape();

void HoldUnstableOrbit(double radius);

#endif