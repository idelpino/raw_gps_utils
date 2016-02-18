#include <iostream>
#include <iomanip>
#include <math.h>

using namespace std;

// WGS84 ellipsoid constants
double a = 6378137; // radius
double e = 8.1819190842622e-2;  // eccentricity

double asq = pow(a,2);
double esq = pow(e,2);

void ecef2lla(double ecef[], double lla[])
{
    double x = ecef[0];
    double y = ecef[1];
    double z = ecef[2];

    double b = sqrt( asq * (1-esq) );
    double bsq = pow(b,2);
    double ep = sqrt( (asq - bsq)/bsq);
    double p = sqrt( pow(x,2) + pow(y,2) );
    double th = atan2(a*z, b*p);

    double lon = atan2(y,x);
    double lat = atan2( (z + pow(ep,2)*b*pow(sin(th),3) ), (p - esq*a*pow(cos(th),3)) );
    double N = a/( sqrt(1-esq*pow(sin(lat),2)) );
    double alt = p / cos(lat) - N;

    // mod lat to 0-2pi
    lon = fmod(lon, (2*M_PI));

    // correction for altitude near poles left out.

    lat = lat * 180 / M_PI;
    lon = lon * 180 / M_PI;

    lla[0] = lat;
    lla[1] = lon;
    lla[2] = alt;

}

int main(int argc, char** argv)
{
    double ecef[] = {
        4789389.9061,
        176941.336569,
        4194510.63987
    };
    double lla[3];

    ecef2lla(ecef, lla);



    cout << setprecision(15);
    cout << "lat: " << lla[0] << endl;
    cout << "lon: " << lla[1] << endl;
    cout << "alt: " << lla[2] << endl;


}



//    const double a = 6378137;
//    const double f = 1/298.257223563;
//    const double b = a*(1-f);
//    const double e = sqrt((pow(a, 2) - pow(b, 2)) / pow(a, 2));
//    const double e1 = sqrt((pow(a, 2) - pow(b, 2)) / pow(b, 2));
//    const double p = sqrt(pow(x, 2) + pow(y, 2));
//    const double theta = atan((z*a)/(p*b));

//    double lambda, phi, h;

//    lambda = atan(y/x);
//    phi = atan((z + pow(e1, 2) * b * pow(sin(theta), 3))/(p - pow(e, 2) * a * pow(cos(theta), 3)));
//    const double N = a / sqrt(1 - pow(e, 2) * pow(sin(phi), 2));
//    h = p/cos(phi) - N;



