#include "orbit_transfers.h"

extern bool HeightsUpdated;
extern double MoonDistance;
extern bool MarsSphereOfInfluence;
int dir;
int iterations = 0;
double CircularOrbitVelocity = 0.0;
double MostImportantMass;
bool MoonApproachStarted = false;

double CurAngle;
double AngleToStartBurn;
double FuelToBurn;

void InitialiseOrbitTransfers()
{
    iterations = 0;
}

void OrbitChangeBurner()
{
    if (OrbitChangeBurn)
    {
        if (Planned_Fuel_Left < fuel)
        {
            throttle = 1.0;
        }
        else
        {
            OrbitChangeBurn = false;
            ClearHeights(); // New orbit, new heights
            throttle = 0.0;
        }
    }
}

double calculateNewVApogee(double Apogee, double NewPerigee)
{
    return sqrt(GRAVITY * MostImportantMass * (2 * NewPerigee) / 
                        (Apogee * (NewPerigee + Apogee)));
}

double calculateNewVPerigee(double Perigee, double NewApogee)
{
    return sqrt(GRAVITY * MostImportantMass * (2 * NewApogee) / 
                        (Perigee * (NewApogee + Perigee)));
}

double rocketEquationForFuel(double deltaV)
{
    // ln(m1/m2) * Vout = deltaV
    // exp(deltaV/Vout) = m1/m2
    // deltaM = fuelToBurn = m1(exp(-deltaV/Vout) - 1)
    // m1 = LANDERMASS, m2 = LANDERMASS - fuelNeeded
    // maxFuelForce / maxFuelBurnRate = impulsePerLitre
    // impulseNeeded / impulsePerLitre = fuelNeeded
    // thrust = dm/dt * Vout, Vout = maxThrust / fuelRateAtMaxThrust (kg/s)
    double Vout = MAX_THRUST / (FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY);
    double deltaM = LANDERMASS / exp(deltaV/Vout) - LANDERMASS;
    return deltaM; // We want to know how much fuel to burn
}

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee)
{
    double v1 = calculateNewVApogee(Apogee, Perigee);
    double v2 = calculateNewVApogee(Apogee, NewPerigee);
    return rocketEquationForFuel(v2 - v1);
}

double calculateFuelBurnedForNewApogee(double Apogee, double Perigee, double NewApogee)
{
    double v1 = calculateNewVPerigee(Perigee, Apogee);
    double v2 = calculateNewVPerigee(Perigee, NewApogee);
    return rocketEquationForFuel(v2 - v1);
}

void PlanDeorbitIfInPermanentOrbit()
{
    if (HeightsUpdated) // This means we are in a permanent orbit as
                       // otherwise perigee and apogee wouldn't have been measured
    {
        // Put us into a landing orbit
        double NewPerigee = EXOSPHERE * 0.6 + MARS_RADIUS; // To use drag to slow us down
        if (position.abs() > Greatest_Height * 0.99)
        {
            double fuelToBurn = calculateFuelBurnedForNewPerigee(Greatest_Height, 
                                                                Lowest_Height, NewPerigee);
            Planned_Fuel_Left = fuel - (fuelToBurn / (FUEL_CAPACITY * FUEL_DENSITY));
            OrbitChangeBurn = Planned_Fuel_Left > 0.0;
            ClearHeights();
        }
    }
}

void Deorbit()
{
    PlanDeorbitIfInPermanentOrbit(); // Sets OrbitChangeBurn to true if a deorbit
                                    // is possible with remaining fuel
    OrbitChangeBurner();
}

void ChangeApogee(double NextApogee)
{
    if (position.abs() <= Lowest_Height * 1.001 && (done & NEXTAPOGEEMET) == 0)
    {
        if ((done & ORBITCHANGECALCDONE) == 0)
        {
            double fuelToBurn = calculateFuelBurnedForNewApogee(Greatest_Height, Lowest_Height, NextApogee);
            Planned_Fuel_Left = fuel - (abs(fuelToBurn) / (FUEL_CAPACITY * FUEL_DENSITY));
            done |= ORBITCHANGECALCDONE;
            dir = 1 - 2 * signbit(fuelToBurn);
        }
        OrbitChangeBurn = Planned_Fuel_Left > 0.0;
        FaceDirection(velocity.norm() * dir);
        OrbitChangeBurner();
        if (fuel <= Planned_Fuel_Left)
        {
            ClearHeights();
            done &= !ORBITCHANGECALCDONE;
            done |= NEXTAPOGEEMET;
        }
    }
}

void ChangePerigee(double NextPerigee)
{
    if (position.abs() >= Greatest_Height * 0.999) // Only run the burner when we are very close to apogee
    {
        if ((done & ORBITCHANGECALCDONE) == 0)
        {
            double fuelToBurn = calculateFuelBurnedForNewPerigee(Greatest_Height, // set to negative as we are burning in direction of travel
                                                            Lowest_Height, NextPerigee);
            Planned_Fuel_Left = fuel - (abs(fuelToBurn) / (FUEL_CAPACITY * FUEL_DENSITY));
            done |= ORBITCHANGECALCDONE;
            dir = 1 - 2 * signbit(fuelToBurn);
            iterations++;
        }
        OrbitChangeBurn = Planned_Fuel_Left > 0.0;
        FaceDirection(velocity.norm() * dir);
        OrbitChangeBurner();
        if (fuel <= Planned_Fuel_Left)
        {
            ClearHeights();
            done &= !ORBITCHANGECALCDONE;
            done |= NEXTPERIGEEMET;
        }
    }
}

void CirculariseCurrentOrbit()
{
    if (HeightsUpdated && (Greatest_Height - Lowest_Height) / Greatest_Height > 0.01) // We have collected current data on Apogee and Perigee and the height difference is outside of error margin
    {
        ChangePerigee(Greatest_Height); // Burns near Apogee to raise Perigee to be roughly equal
    }
}

void MoveToOrbitInPlane(double NextApogee, double NextPerigee)
{
    if (HeightsUpdated)
    {
        if ((done & NEXTAPOGEEMET) == 0)
        {
            ChangeApogee(NextApogee);
        }
        else
        {
            ChangePerigee(NextPerigee);
        }
    }
}

double CalculateAngleXY(const vector3d& a, const vector3d& b, const vector3d& c) // finds the angle between AB and BC projected into the XY plane counterclockwise
{
    vector3d A = a - b; A.z = 0;
    vector3d C = c - b; C.z = 0;
    vector3d B = c - a; B.z = 0;
    double CosTheta = (A.abs2() + C.abs2() - B.abs2()) / (2 * A.abs() * C.abs());
    double Theta = acos(CosTheta);
    if (A.crossProduct(C).z < 0.0) Theta = 2 * M_PI - Theta;
    return Theta;
}

double KeplerPeriod(double SemiMajorAxis)
{
    return 2 * M_PI * sqrt(pow(SemiMajorAxis, 3.0) * pow(GRAVITY * MostImportantMass, -1.0));
}

// @brief Entry conditions: circular orbit around Mars/Main body, velocity in this orbit measured, coplanar with moon
void ApproachMoon()
{
    if (!MoonApproachStarted && HeightsUpdated)
    {
        CurAngle = CalculateAngleXY(position, vector3d(), MoonPos);
        double ApogeeHeight = MOONAPROACHAPOGEE;
        FuelToBurn = calculateFuelBurnedForNewApogee((Greatest_Height + Lowest_Height) / 2.0, (Greatest_Height + Lowest_Height) / 2.0, ApogeeHeight) / (FUEL_CAPACITY * FUEL_DENSITY);
        double JourneyTime = KeplerPeriod(ApogeeHeight / 2.0 + ((Greatest_Height + Lowest_Height) / 2.0) / 2.0) / 2.0;
        // Account for Gravitational acceleration by moon by multiplying MaxJourneyTime by < 1.0
        double AngleToBurn = M_PI - JourneyTime * MOONOMEGA; // Angle between lander and moon to place our perigee for the transfer
        AngleToBurn += (MOONRADIUS) / (MoonDistance * 2 * M_PI); // Account for radius in case moon is close to planet
        while (AngleToBurn < 0) {
        AngleToBurn += 2 * M_PI;
        }
        while (CurAngle < 0) {
            CurAngle += 2 * M_PI;
        }
        double TimeToBurn = FuelToBurn / FUEL_RATE_AT_MAX_THRUST;
        AngleToStartBurn = AngleToBurn - TimeToBurn * CircularOrbitVelocity * 0.5 / ((Greatest_Height + Lowest_Height) / 2.0);
        MoonApproachStarted = (CurAngle - AngleToStartBurn) <= M_PI * 0.005 && (CurAngle - AngleToStartBurn) >= 0.0; // when we are within 1/200 of a radian
        if (MoonApproachStarted && abs(FuelToBurn) < fuel)
        {
            OrbitChangeBurn = true;
            Planned_Fuel_Left = fuel - abs(FuelToBurn);
        }
    }
    if (MoonApproachStarted)
    {
        FaceDirection(-velocity.norm()); // Throughout this function we have assumed the lander is in a lower orbit than the moon, so we will continue to do so
        OrbitChangeBurner();
    }
}

bool EscapePrevented = false;
void PreventMoonEscape()
{
    if (!EscapePrevented)
    {

    }
    else
    {
        done |= MOONESCAPEPREVENTED;
    }
}
