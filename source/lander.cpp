// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "all.h"

vector3d positionNMinus1;
using namespace std;

#define USEVERLET

COPILOT_ACTION AUTO_NEXT;
COPILOT_ACTION TEMPCOMMAND;
vector3d MoonPos;
vector3d MoonRelPos;

int CustomOrbitInput [2 * INPUTRESOLUTION + 2] = { 0 }; // extra two spaces are for the exponents
int CurrentSelection;
double InputApogee;
double InputPerigee;
bool MoonGravityEnabled;
double MoonDistance;
double Altitude;
double DistanceFromMostImportantMass;

void autopilot (void)
{
  if (simulation_time > 5.0) // Gives it some time to stabilise before we measure and use data
  {
    UpdateHeights();
    PreventLanderEscape(); // We don't want to fly away forever
    switch (AUTO_NEXT)
    {
      case (PROPORTIONALLANDING):
        FaceDirection(-velocity.norm());
        Deorbit();
        LandProportional();
        break;

      case (SUICIDELANDING):
        FaceDirection(-velocity.norm());
        Deorbit();
        LandSuicide();
        break;

      case (CIRCULARISEORBIT): // Does the most efficient orbit circularisation by raising perigee from apogee
        CirculariseCurrentOrbit();
        break;

      case (CUSTOMORBIT):
        if (!(AUTO_NEXT == TAKINGINPUT))
        {
          MoveToOrbitInPlane(InputApogee, InputPerigee); // Does a Hoffman transfer when data has been collected
        }
        break;

      case (GOTOMOON):
        ApproachMoon();
        break;

      case (BIIMPULSIVEMOONTRANSFER):
        ApproachMoon();
        if (MoonApproachStarted && (!OrbitChangeBurn || (done & MOONAPROACHBURNFINISHED) == MOONAPROACHBURNFINISHED))
        {
          done |= MOONAPROACHBURNFINISHED; // status of Orbit_ChangeBurn will change once we start burning in here, hence the done flag
          if (MoonAltitude < (MOONRADIUS + EXOSPHERE) * 1.2 && !MarsSphereOfInfluence) // We are close and Moon gravity is stronger than Mars gravity
          {
            if ((done & MOONESCAPEPREVENTED) != MOONESCAPEPREVENTED) PreventMoonEscape(); // brings us into a reasonably stable moon orbit
            else CirculariseCurrentOrbit(); // circularises it
          }
        }

      case (DONOTHING):
        break;

      default:
        break;
    }
  }
}

vector3d FGravMars;
vector3d FGravMoon;
vector3d FDragLander;
vector3d FDragChute;
vector3d Thrust;
vector3d acceleration;
double RotationAngle;

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  MoonPos = {-MoonDistance * sin(MOONOMEGA * simulation_time), MoonDistance * cos(MOONOMEGA * simulation_time), 0.0};
  MoonRelPos = MoonPos - position;  

  FGravMars = -position.norm() * ((MARS_MASS * LANDERMASS * GRAVITY) / (position.abs2()));
  if (MoonGravityEnabled)
  {
    FGravMoon = MoonRelPos.norm() * ((MOONMASS * LANDERMASS * GRAVITY) / (MoonRelPos.abs2()));
  }
  MarsSphereOfInfluence = FGravMars.abs2() > FGravMoon.abs2();
  MostImportantMass = MarsSphereOfInfluence ? MARS_MASS:MOONMASS;
  Altitude = MarsSphereOfInfluence ? MarsAltitude:MoonAltitude;
  DistanceFromMostImportantMass = MarsSphereOfInfluence ?  + MARS_RADIUS:MoonAltitude + MOONRADIUS;

  FDragLander = pow(LANDER_SIZE, 2) * DRAGCONSTANT(DRAG_COEF_LANDER) * VELCONSTANT;
  FDragChute = pow(LANDER_SIZE * 2, 2) * 5 * DRAGCONSTANT(DRAG_COEF_CHUTE) * VELCONSTANT;
  Thrust = thrust_wrt_world();
  acceleration = (FGravMars + FGravMoon + FDragLander + Thrust) / (double)LANDERMASS;
  FaceDirection(-VecAtAngleToPosInPlane(RotationAngle));

#if defined(USEVERLET)
  if (simulation_time == 0)
  {  positionNMinus1 = position - velocity * delta_t - acceleration * delta_t * delta_t; }
  vector3d PositionTemp = position;
  position = 2 * PositionTemp - positionNMinus1 + (delta_t * delta_t) * acceleration;
  positionNMinus1 = PositionTemp;
  velocity = (position - positionNMinus1) / delta_t;
#else // Do an euler integration
  position += velocity * delta_t;
  velocity += acceleration * delta_t;
#endif

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) 
  {
    stabilized_attitude = false;
    autopilot();
  }
  else
  {
    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
  }

}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  InitialiseSpecialFunc();
  InitialiseOrbitTransfers();
  ResetHelperInfo();
  AUTO_NEXT = DONOTHING;
  RotationAngle = 0.0;
  CurrentSelection = 0;
  MoonGravityEnabled = false;
  MoonDistance = 20 * MARS_RADIUS;
  orbital_zoom = 0.15;
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "geostationary orbit";
  scenario_description[7] = "";
  
  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS * 1, 0.0, 0.0);
    velocity = vector3d(0.0, 3247.087385863725 * pow(1, -0.5), 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    MoonDistance = 36000000;
    orbital_zoom = 0.22;
    break;

  case 1:
    // a descent from rest at 10km MarsAltitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    position = vector3d(20428000.0, 0.0, 0.0);
    velocity = vector3d(0.0, 1446.0, 0.0);
    orientation = vector3d(0.0, 180.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
  Lowest_Height = position.abs();
  Greatest_Height = position.abs();
  set_orbital_projection_matrix();
  refresh_all_subwindows();
}
