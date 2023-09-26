#include "orbit_transfers.h"

extern bool HeightsUpdated;
extern double MoonDistance;
extern bool MarsSphereOfInfluence;
int iterations = 0;
double MostImportantMass;
bool MoonApproachStarted = false;

double CurAngle;
double AngleToStartBurn;
double FuelToBurn = 0.0;
double ApogeeHeight = 0.0;
double VelStart;
double VelAim;
vector3d OrbitVelTangential;
vector3d OrbitVelNormal;
double MoonApproachPerigee;
double OrbitHeight;
bool EscapePrevented = false;
extern bool EscapePreventionStarted;

void InitialiseOrbitTransfers()
{
    iterations = 0;
    MoonApproachStarted = false;
    FuelToBurn = 0.0;
    ApogeeHeight = 0.0;
    VelStart = 0.0;
    VelAim = 0.0;
    OrbitHeight = 0.0;
    EscapePrevented = false;
    EscapePreventionStarted = false;
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
    OrbitChangeBurnerVel(OrbitVel.norm());
}

void OrbitChangeBurnerVel(vector3d dir) // dir is a unit vector
{
    vector3d VelResolved = (OrbitVel * dir) * dir;
    if (OrbitChangeBurner)
    {
        if (VelStart > VelAim)
        {
            FaceDirection(-dir);
            OrbitChangeBurnerVelDecrease(VelResolved);
        }
        else
        {
            FaceDirection(dir);
            OrbitChangeBurnerVelIncrease(VelResolved);
        }
    }
}

double VisVivaEquation(double Apogee, double Perigee, double CurDist) // a lil vis-viva equation
{
    double mu = GRAVITY * MostImportantMass;
    double e = (Apogee - Perigee) / (Apogee + Perigee);
    double a = 0.5 * (Perigee + Apogee);
    return sqrt(mu * (2.0 / CurDist - 1 / a));
}

double RAtThetaEllipse(double a, double e, double theta)
{
    return a * (1 - e * e) / (1 + e * cos(theta));
}

double rocketEquationForFuel(double v2, double v1)
{
    double fuelRate = (FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY);
    double Mfinal = LANDERMASS * (log(MAX_THRUST + fuelRate * v2) - log(MAX_THRUST + fuelRate * v1));
    return Mfinal; // return how much fuel to burn (if negative v2 < v1)
}

double calculateFuelBurnedForNewPerigee(double Apogee, double Perigee, double NewPerigee)
{
    double v1 = VisVivaEquation(Apogee, Perigee, Apogee);
    double v2 = VisVivaEquation(Apogee, NewPerigee, Apogee);
    return rocketEquationForFuel(v2, v1);
}

double calculateFuelBurnedForNewApogee(double Apogee, double Perigee, double NewApogee)
{
    double v1 = VisVivaEquation(Apogee, Perigee, Perigee);
    double v2 = VisVivaEquation(NewApogee, Perigee, NewApogee);
    return rocketEquationForFuel(v2, v1);
}

extern bool descending;
extern bool previous_descending;
void Deorbit()
{
    if (descending && !previous_descending)
    {
        double curV = velocity.abs();
        double NewPerigee;
        if (MarsSphereOfInfluence)
        {
            NewPerigee = 0.55 * EXOSPHERE + MARS_RADIUS;
            if (MoonGravityEnabled)
            {
                // take into account moon cantankerous interference on perigee
                NewPerigee -= 0.0004 * 0.5 * (FGravMoon * velocity.norm()) * pow(KeplerPeriod((Greatest_Height + Lowest_Height) * 0.5), 2);
            }
        }
        else { NewPerigee = 0.9 * LocalRadius; }
        double maxV = VisVivaEquation(Altitude + LocalRadius, NewPerigee, Altitude + LocalRadius);
        if (maxV < curV)
        {
            VelAim = maxV;
            VelStart = curV;
            OrbitChangeBurn = true;
        }
    }
    OrbitChangeBurnerVel();
}

void ChangeApogee(double NextApogee)
{
    if (Altitude + LocalRadius <= Lowest_Height * 1.01 && (done & NEXTAPOGEEMET) == 0)
    {
        if ((done & ORBITCHANGECALCDONE) == 0)
        {
            VelStart = OrbitVel.abs();
            VelAim = VisVivaEquation(NextApogee, Altitude + LocalRadius, Altitude + LocalRadius);
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
    if (Altitude + LocalRadius >= Greatest_Height * 0.99 && (done & NEXTPERIGEEMET) == 0) // Only run the burner when we are very close to apogee
    {
        if ((done & ORBITCHANGECALCDONE) == 0)
        {
            VelStart = OrbitVel.abs();
            VelAim = VisVivaEquation(Altitude + LocalRadius, NextPerigee, Altitude + LocalRadius);
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
    if (HeightsUpdated && (Greatest_Height - Lowest_Height) / Greatest_Height > 0.02) // We have collected current data on Apogee and Perigee and the height difference is outside of error margin
    {
        if (!OrbitChangeBurn)
        { done &= !NEXTPERIGEEMET; }
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

double TimeToReachMoon(double ApogeeHeight, double MoonHeight, double OriginalRadius, double velocity)
{
    double L = pow(OriginalRadius * velocity, 2) / (GRAVITY * MARS_MASS);
    double a = ApogeeHeight * 0.5 + OriginalRadius * 0.5;
    double b = sqrt(L * a);
    double e = sqrt(1 - (b/a) * (b/a));
    double AreaEllipse = M_PI * a * b;
    double ThetaCircle = asin((MoonHeight + OriginalRadius - a)/a);
    double ThetaEllipse = atan(MoonHeight / 
                            (b / a *  sqrt(a * a - pow((MoonHeight + OriginalRadius - a), 2))));
    double ThetaCarvedByJourney = M_PI * 0.5 + ThetaEllipse;
    double EccentricAnomalyE = 2 * atan(sqrt((1 - e)/(1 + e)) * tan(ThetaCarvedByJourney * 0.5));
    double MeanAnomaly = EccentricAnomalyE - e * sin(EccentricAnomalyE);
    double AreaJourney = 0.5 * a * b * MeanAnomaly;
    double JourneyTime = KeplerPeriod(a) * AreaJourney / AreaEllipse;
    return JourneyTime;
}

double CalculateBurnTime(double deltaV)
{
    double massLossRate = FUEL_RATE_AT_MAX_THRUST * FUEL_DENSITY;
    double exponentConst = exp(-deltaV * massLossRate / MAX_THRUST);
    return LANDERMASS * (1 - exponentConst) / massLossRate;
}

// Entry conditions: circular orbit around Mars/Main body, velocity in this orbit measured, coplanar with moon
void ApproachMoon(double OrbitHeight)
{
    if (!MoonApproachStarted && HeightsUpdated)
    {
        CurAngle = CalculateAngleXY(position, vector3d(), MoonPos);
        ApogeeHeight = MoonDistance + 5 * MOONRADIUS;
        VelAim = VisVivaEquation(ApogeeHeight, OrbitHeight, OrbitHeight);
        VelStart = velocity.abs();

        //solve for journey time based on keplers laws assuming no moon gravity
        double L = pow(OrbitHeight * velocity.abs(), 2) / (GRAVITY * MostImportantMass);
        double a = ApogeeHeight * 0.5 + OrbitHeight * 0.5;
        double b = sqrt(L * a);
        double e = sqrt(1 - (b/a) * (b/a));
        double AreaEllipse = M_PI * a * b;
        double ThetaCircle = asin((MoonDistance + OrbitHeight - a)/a);

        double Correction = MoonDistance + MOONRADIUS;// - MOONRADIUS * pow(MOONRADIUS / MoonDistance, -0.05);

        double JourneyTime = TimeToReachMoon(ApogeeHeight, Correction, Lowest_Height, velocity.abs());

        AngleToStartBurn = M_PI - (JourneyTime) * MOONOMEGA; // Angle between lander and moon to place our perigee for the transfer
        double BurnTime = CalculateBurnTime(VelAim - VelStart);
        double LanderOmega = velocity.abs() / position.abs();
        AngleToStartBurn += BurnTime * LanderOmega * 0.5; // from lower energy orbits the burn time can be significant enough to break this function
        AngleToStartBurn += 1.5 * MOONRADIUS / MoonDistance; // We want to show up slightly behind the moon and boost into an orbit with it


        //double ThetaEllipse = atan(MoonDistance /  // Not necessary really, and not always reliable
        //                        (b / a *  sqrt(a * a - pow((MoonDistance + OrbitHeight - a), 2))));
        //double ThetaAtPerigee = atan((MoonDistance + OrbitHeight) / (b * cos(ThetaEllipse)));
        //AngleToStartBurn -= (M_PI / 2 - ThetaEllipse); // This is to account for wider ellipses causing an encounter earlier than if we appeared opposite to the burn point
        while (AngleToStartBurn < 0) {
        AngleToStartBurn += 2 * M_PI;
        }
        while (CurAngle < 0) {
            CurAngle += 2 * M_PI;
        }
        MoonApproachStarted = (CurAngle - AngleToStartBurn) <= M_PI * 0.005 && (CurAngle - AngleToStartBurn) >= 0.0; // when we are within 1/200 of a radian
        if (MoonApproachStarted)
        {
            OrbitChangeBurn = true;
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

bool PreventMoonEscape()
{
    if (EscapePrevented) return true;
    if ((MoonApproachStarted && !OrbitChangeBurn) || (done & MOONAPROACHBURNFINISHED) == MOONAPROACHBURNFINISHED)
    {
        double CurPerigeeCalculated = MoonApproachPerigee;
        // HyperbolicPerigee function isn't very accurate in terms of the number provided, likely due to the mars
        // gravity being significant in all scenarios but is still very good at telling when we have reached
        // the perigee as the number starts to rise once we pass it.
        MoonApproachPerigee = abs(HyperbolicPerigee());
        done |= MOONAPROACHBURNFINISHED; // status of Orbit_ChangeBurn will change once we start burning in here, hence the done flag
        MoonApproachStarted = false;
        if ((MoonApproachPerigee > CurPerigeeCalculated) && Altitude < 3 * MOONRADIUS && !MarsSphereOfInfluence)
        {
            double dotProd = OrbitVel * (-MoonRelPos);
            double scalarProjection = dotProd / (MoonRelPos.abs() * OrbitVel.abs());
            OrbitVelNormal = OrbitVel * scalarProjection;
            OrbitVelTangential = OrbitVel - OrbitVelNormal;
            if ((done & ORBITCHANGECALCDONE) == 0)
            { 
                OrbitHeight = LocalRadius + Altitude;
                VelStart = OrbitVelTangential.abs();
                VelAim = VisVivaEquation(OrbitHeight, OrbitHeight, OrbitHeight);
                velocity = OrbitVelTangential + MoonVel;
                OrbitChangeBurn = true;
                done |= ORBITCHANGECALCDONE;
            }
            // The normal component of the velocity can be largely ignored as we approach the moon at a sharp angle
            OrbitChangeBurnerVel(OrbitVelTangential.norm());
            if (!OrbitChangeBurn)
            {
                ClearHeights();
                done &= !ORBITCHANGECALCDONE;
                EscapePrevented = true;
            }
        }
    }
    return EscapePrevented;
}

void HoldUnstableOrbit(double radius)
{
    if (Altitude + LocalRadius > radius * 1.2) { ChangeApogee(radius); } // make big changes, otherwise fuel is wasted or worst case, the orbit is broken
    if (Altitude + LocalRadius < radius * 1.2) { ChangePerigee(radius); }
}

extern double climb_speed;
void KillNormalVel()
{
    double MaxMarsAffect = FGravMars.abs() / LANDERMASS;
    if (abs(climb_speed) > MaxMarsAffect * 1.5)
    {
        VelAim = 0;
        VelStart = (OrbitVel * -MoonRelPos.norm());
        OrbitChangeBurn = true;
    }
    OrbitChangeBurnerVel(-MoonRelPos.norm());
}
/*
bool ApogeeChangeQueued = false;
bool PerigeeChangeQueued = false;
extern bool descending;
extern bool previous_descending;
void HoldUnstableOrbit(double radius)
{
    if (Altitude > (radius - LocalRadius) * 1.05) { ChangeApogee(radius); } // make big changes, otherwise fuel is wasted or worst case, the orbit is broken
    if (Altitude < (radius - LocalRadius) / 1.05) { ChangePerigee(radius); }
    return;
    double ROppositeIfAtApoOrPerigee = (Altitude + LocalRadius) / 
                    (1.0 - (2 * GRAVITY * MostImportantMass / (OrbitVel.abs2() * Altitude + LocalRadius)));
    if (!descending && previous_descending) // we have just gone past the lowest point in the orbit
    {
        if (ROppositeIfAtApoOrPerigee < radius || ROppositeIfAtApoOrPerigee > radius * 1.01)
        {
            OrbitChangeBurn = true;
            VelAim = VisVivaEquation(radius, Altitude + LocalRadius, Altitude + LocalRadius);
        }
    }
    else if (descending && !previous_descending) // we have just gone past the highest point in the orbit
    {
        if (ROppositeIfAtApoOrPerigee < radius || ROppositeIfAtApoOrPerigee > radius * 1.01)
        {
            OrbitChangeBurn = true;
            VelAim = VisVivaEquation(Altitude + LocalRadius, radius, Altitude + LocalRadius);
        }        
    }
    OrbitChangeBurnerVel();
}*/