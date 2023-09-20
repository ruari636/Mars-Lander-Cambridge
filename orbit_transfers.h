#include "lander_special_func.h"

#ifndef ORBIT_TRANSFER_H
#define ORBIT_TRANSFER_H

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

#endif