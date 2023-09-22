#include "orbit_transfers.h"

extern bool HeightsUpdated;
extern double MoonDistance;
extern bool MarsSphereOfInfluence;
int iterations = 0;
double CircularOrbitVelocity = 0.0;
double MostImportantMass;
bool MoonApproachStarted = false;

double CurAngle;
double AngleToStartBurn;
double FuelToBurn;
double ApogeeHeight;
double VelStart;
double VelAim;

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

void OrbitChangeBurnerVelIncrease(vector3d OrbitVel)
{
    if (OrbitChangeBurn)
    {
        if (OrbitVel.abs() < VelAim)
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

void OrbitChangeBurnerVelDecrease(vector3d OrbitVel)
{
    if (OrbitChangeBurn)
    {
        if (OrbitVel.abs() > VelAim)
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

void OrbitChangeBurnerVel()
{
    vector3d OrbitVel;
    if (MarsSphereOfInfluence)
    {
        OrbitVel = velocity;
    }
    else
    {
        OrbitVel = velocity - MoonVel;
    }
    if (VelStart > VelAim)
    {
        FaceDirection(-OrbitVel.norm());
        OrbitChangeBurnerVelDecrease(OrbitVel);
    }
    else
    {
        FaceDirection(OrbitVel.norm());
        OrbitChangeBurnerVelIncrease(OrbitVel);
    }
}

void OrbitChangeBurnerVel(vector3d dir)
{
    vector3d OrbitVel;
    if (MarsSphereOfInfluence)
    {
        OrbitVel = velocity;
    }
    else
    {
        OrbitVel = velocity - MoonVel;
    }
    OrbitVel = (OrbitVel * dir) * dir;
    if (VelStart > VelAim)
    {
        FaceDirection(-OrbitVel.norm());
        OrbitChangeBurnerVelDecrease(OrbitVel);
    }
    else
    {
        FaceDirection(OrbitVel.norm());
        OrbitChangeBurnerVelIncrease(OrbitVel);
    }
}

double calculateNewV(double Apogee, double Perigee, double CurDist) // a lil vis-viva equation
{
    double mu = GRAVITY * MostImportantMass;
    double e = (Apogee - Perigee) / (Apogee + Perigee);
    double a = 0.5 * (Perigee + Apogee);
    return sqrt(mu * (2.0 / CurDist - 1 / a));
}

double rocketEquationForFuel(double v2, double v1)
{
    // mv + F * deltaT = (m - mDelta)(v + vDelta)
    // mDelta = deltaT * fuelRate (kg/s)
    // mv + F * deltaT = (m - deltaT * fuelRate)(v + vDelta) = mv + mvDelta - deltaT * fuelRate * v - deltaT * fuelRate * vDelta
    // F * deltaT = mvDelta - deltaT * fuelRate * v - deltaT * fuelRate * vDelta 
    // (F + fuelRate * v + fuelRate * vDelta) * deltaT = mvDelta (next take limits/differential form)
    // (F + fuelRate * v + fuelRate * dv) * dt = mdv = Fdt + fuelRate * v * dt + fuelRate * dvdt (next assume dvdt = 0)
    // mdv = (F + fuelRate * v)dt
    // dt = (m / (F + fuelRate * v))dv
    // t2 - t1 = (m/fuelRate) * (ln(F + fuelRate * v2) - ln(F + fuelRate * v1))
    // t2 - t1 = (m/fuelRate) * ln( (F + fuelRate * v2) / (F + fuelRate * v1) )
    // fuelBurned = fuelRate(t2 - t1) = m * ln( (F + fuelRate * v2) / (F + fuelRate * v1) )
    double fuelRate = (FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY);
    double Mfinal = LANDERMASS * (log(MAX_THRUST + fuelRate * v2) - log(MAX_THRUST + fuelRate * v1));
    return Mfinal; // return how much fuel to burn (if negative v2 < v1)
}

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee)
{
    double v1 = calculateNewV(Apogee, Perigee, Apogee);
    double v2 = calculateNewV(Apogee, NewPerigee, Apogee);
    return rocketEquationForFuel(v2, v1);
}

double calculateFuelBurnedForNewApogee(double Apogee, double Perigee, double NewApogee)
{
    double v1 = calculateNewV(Apogee, Perigee, Perigee);
    double v2 = calculateNewV(NewApogee, Perigee, NewApogee);
    return rocketEquationForFuel(v2, v1);
}

void PlanDeorbitIfInPermanentOrbit()
{
    if (HeightsUpdated) // This means we are in a permanent orbit as
                       // otherwise perigee and apogee wouldn't have been measured
    {
        // Put us into a landing orbit
        double NewPerigee = EXOSPHERE * 0.5 + MARS_RADIUS; // To use drag to slow us down
        if (position.abs() > Greatest_Height * 0.99 && !OrbitChangeBurn)
        {
            // double fuelToBurn = -calculateFuelBurnedForNewPerigee(Greatest_Height, 
            //                                                     Lowest_Height, NewPerigee);
            // Planned_Fuel_Left = fuel - (fuelToBurn / (FUEL_CAPACITY * FUEL_DENSITY));
            NewPerigee -= 0.0004 * 0.5 * (FGravMoon * velocity.norm()) * pow(KeplerPeriod((Greatest_Height + Lowest_Height) * 0.5), 2); // take into account moon cantankerous interference on perigee
            VelStart = velocity.abs();
            VelAim = calculateNewV(Greatest_Height, NewPerigee, Greatest_Height);
            OrbitChangeBurn = true;
            ClearHeights();
        }
    }
}

void Deorbit()
{
    PlanDeorbitIfInPermanentOrbit(); // Sets OrbitChangeBurn to true if a deorbit
                                    // is possible with remaining fuel
    OrbitChangeBurnerVel();
}

void ChangeApogee(double NextApogee)
{
    if (position.abs() <= Lowest_Height * 1.001 && (done & NEXTAPOGEEMET) == 0)
    {
        if ((done & ORBITCHANGECALCDONE) == 0)
        {
            VelStart = velocity.abs();
            VelAim = calculateNewV(NextApogee, Lowest_Height, Lowest_Height);
            done |= ORBITCHANGECALCDONE;
            OrbitChangeBurn = true;
        }
        OrbitChangeBurnerVel();
        if (!OrbitChangeBurn)
        {
            ClearHeights();
            done &= !ORBITCHANGECALCDONE;
            done |= NEXTAPOGEEMET;
        }
    }
}

void ChangePerigee(double NextPerigee)
{
    if (position.abs() >= Greatest_Height * 0.999 && (done & NEXTPERIGEEMET) == 0) // Only run the burner when we are very close to apogee
    {
        if ((done & ORBITCHANGECALCDONE) == 0)
        {
            VelStart = velocity.abs();
            VelAim = calculateNewV(Greatest_Height, NextPerigee, Greatest_Height);
            done |= ORBITCHANGECALCDONE;
            OrbitChangeBurn = true;
        }
        OrbitChangeBurnerVel();
        if (!OrbitChangeBurn)
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

void MoveToOrbitInPlane(double NextApogee, double NextPerigee) // This is done as efficiently as possible; if the new Apogee is higher than the current one we raise apogee first as
                                                               // changing the perigee requires less impulse from a greater height
{
    if (HeightsUpdated)
    {
        if (NextApogee > Greatest_Height)
        {
            if ((done & NEXTAPOGEEMET) == 0)
            {
                ChangeApogee(NextApogee);
            }
            else if ((done & NEXTPERIGEEMET) == 0)
            {
                ChangePerigee(NextPerigee);
            }
        }
        else if (NextApogee <= Greatest_Height)
        {
            if ((done & NEXTPERIGEEMET) == 0)
            {
                ChangePerigee(NextPerigee);
            }
            else if ((done & NEXTAPOGEEMET) == 0)
            {
                ChangeApogee(NextApogee);
            }
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
        ApogeeHeight = MoonDistance + 2 * MOONRADIUS;
        VelAim = calculateNewV(ApogeeHeight, Lowest_Height, Lowest_Height);
        double JourneyTime = KeplerPeriod((MoonDistance - MOONRADIUS) / 2.0 + ((Greatest_Height + Lowest_Height) / 2.0) / 2.0) / 2.0; // use the moon distance instead of our calculated apogee for the apogee of the journey
        // TODO: Account for Gravitational acceleration by moon by multiplying MaxJourneyTime by < 1.0
        double AngleToBurn = M_PI - JourneyTime * MOONOMEGA; // Angle between lander and moon to place our perigee for the transfer
        double LanderOmega = velocity.abs() / position.abs();
        AngleToBurn += LanderOmega * MOONRADIUS * 0.9 / (MOONOMEGA * MoonDistance * M_PI); // Account for radius in case moon is close to planet (but under account slightly for moons acceleration of lander)
        while (AngleToBurn < 0) {
        AngleToBurn += 2 * M_PI;
        }
        while (CurAngle < 0) {
            CurAngle += 2 * M_PI;
        }
        double TimeToBurn = FuelToBurn * (FUEL_CAPACITY * FUEL_DENSITY) / FUEL_RATE_AT_MAX_THRUST;
        AngleToStartBurn = AngleToBurn - TimeToBurn * CircularOrbitVelocity * 0.5 / ((Greatest_Height + Lowest_Height) / 2.0);
        MoonApproachStarted = (CurAngle - AngleToStartBurn) <= M_PI * 0.005 && (CurAngle - AngleToStartBurn) >= 0.0; // when we are within 1/200 of a radian
        if (MoonApproachStarted)
        {
            OrbitChangeBurn = true;
            VelStart = velocity.abs();
        }
    }
    if (MoonApproachStarted)
    {
        OrbitChangeBurnerVel();
    }
}

double SpecificOrbitalEnergy()
{
    return velocity.abs2() - (GRAVITY * MostImportantMass / position.abs());
}

double Eccentricity()
{
    double GM = GRAVITY * MostImportantMass;
    vector3d RelativeVel = MarsSphereOfInfluence ? velocity:velocity - MoonVel;
    vector3d RadiusVec = MarsSphereOfInfluence ? position:-1 * MoonRelPos;
    double FGrav = MarsSphereOfInfluence ? FGravMars.abs():FGravMoon.abs();
    double a = GM / (2 * GM / LocalRadius + Altitude - velocity.abs2());
    double E = -GM / (2 * a);
    double ReducedMass = LANDERMASS * MostImportantMass / (LANDERMASS + MostImportantMass);
    double Alpha = FGrav * (LocalRadius + Altitude) * (LocalRadius + Altitude);
    vector3d AngMomentum = LANDERMASS * RadiusVec.crossProduct(RelativeVel);
    double e = sqrt(1 + 2 * E * AngMomentum.abs2() / (ReducedMass * Alpha * Alpha));
    return e;
}

double HyperbolicPerigee()
{
    // mu * (2/r - 1/a) = v^2
    // a * 2 * mu / r - mu = a * v^2
    // a * (2 * mu / r - v^2) = mu
    // a = mu / (2 * mu / r - v^2)
    double GM = GRAVITY * MostImportantMass;
    double a = GM / (2 * GM / (LocalRadius + Altitude) - velocity.abs2());
    double e = Eccentricity();
    double periapsis = -a * (e - 1);
    return periapsis;
}

bool EscapePrevented = false;
void PreventMoonEscape()
{
    if (!EscapePrevented)
    {
        if (Eccentricity() > 0.5)
        {
            FaceDirection(-velocity);
            throttle = 1.0;
        }
        else
        {
            throttle = 0.0;
            EscapePrevented = true;
        }
    }
    else
    {
        done |= MOONESCAPEPREVENTED;
    }
}
