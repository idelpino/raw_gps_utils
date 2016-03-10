//
// Created by ptirindelli on 22/02/16.
//
#include <iostream>
#include <iomanip>
#include <math.h>

//includes from Eigen lib
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Sparse>

using namespace std;


int main(int argc, char** argv)
{
    cout << setprecision(12);
    bool checkOnlyTheFirstTransform = true;

    Eigen::Matrix<double, 3, 1> vehicle_p_map(0, 0, 0);//thrird coords always 0! we are in 2D
    double vehicle_o = 90*M_PI/180;
    Eigen::Matrix<double, 3, 1> sensor_p_base(1, 2, 0);

    Eigen::Matrix<double, 3, 1> init_vehicle_p(4789389, 176941, 4194510);
    double init_vehicle_o = 30*M_PI/180;

    std::cout << "----------- originali -----------\n";
    if(!checkOnlyTheFirstTransform) {
        std::cout << "init_vehicle_p: " << init_vehicle_p.transpose() << std::endl;
        std::cout << "init_vehicle_o: " << init_vehicle_o * 180 / M_PI << " [printed in degree]" << std::endl;
    }
    std::cout << "vehicle_p_map: " << vehicle_p_map.transpose() << std::endl;
    std::cout << "vehicle_o_map: " << vehicle_o * 180 / M_PI << " [printed in degree]" << std::endl;
    std::cout << "sensor_p_base: " << sensor_p_base.transpose() << std::endl;
    std::cout << "---------------------------------\n";



    /*
     * Base-to-map transform matrix
     */
    Eigen::Matrix<double, 3, 3> T_base2map = Eigen::Matrix<double, 3, 3>::Identity();
    T_base2map(0, 0) = cos(vehicle_o);
    T_base2map(0, 1) = -sin(vehicle_o);
    T_base2map(1, 0) = sin(vehicle_o);
    T_base2map(1, 1) = cos(vehicle_o);

    Eigen::Matrix<double, 3, 1> sensor_p_map; // sensor position with respect to map frame (initial frame of the experiment)
    sensor_p_map = T_base2map * sensor_p_base + vehicle_p_map;

    std::cout << "\tsensor_p_map: " << sensor_p_map.transpose() << std::endl;

    if(checkOnlyTheFirstTransform)
        return 0;

    /*
     * init_vehicle_p from ecef to lla
     */
    // WGS84 ellipsoid constants
    double a = 6378137; // earth's radius
    double e = 8.1819190842622e-2;  // eccentricity
    double asq = a*a;
    double esq = e*e;
    double b = sqrt( asq * (1)-esq);
    double bsq = b*b;
    double ep = sqrt( (asq - bsq)/bsq);
    double p = sqrt( init_vehicle_p[0]*init_vehicle_p[0] + init_vehicle_p[1]*init_vehicle_p[1] );
    double th = atan2(a*init_vehicle_p[2], b*p);

    double lon = atan2(init_vehicle_p[1],init_vehicle_p[0]);
    double lat = atan2( (init_vehicle_p[2] + ep*ep*b*pow(sin(th),3) ), (p - esq*a*pow(cos(th),3)) );
    //        double N = a/( sqrt(1)-esq*pow(sin(lat),2)) ));
    //        double alt = p / cos(lat) - N);
    // mod lat to 0-2pi
    if(lon<0) lon += 2*M_PI;
    if(lon>2*M_PI) lon += 2*M_PI;

    // correction for altitude near poles left out.

    //convert to degrees
//    lat = lat * 180 / M_PI;
//    lon = lon * 180 / M_PI;

    //std::cout << "init_vehicle_p: " << init_vehicle_p[0] << ", " << init_vehicle_p[1] << ", " << init_vehicle_p[2] << std::endl;
    //        std::cout << "init_vehicle_p LLA: " << lat * 180 / M_PI << ", " << lon * 180 / M_PI /*<< ", " << alt*/ << std::endl;


    /*
     * map-to-ECEF transform matrix
     */
    Eigen::Matrix<double, 3, 3> R1 = Eigen::Matrix<double, 3, 3>::Identity();
    R1(0, 0) = cos(lon);        R1(0, 1) = sin(lon);
    R1(1, 0) = -sin(lon);       R1(1, 1) = cos(lon);

    Eigen::Matrix<double, 3, 3> R2 = Eigen::Matrix<double, 3, 3>::Identity();
    R2(0, 0) = cos(lat);        R2(0, 2) = sin(lat);
    R2(2, 0) = -sin(lat);       R2(2, 2) = cos(lat);

    Eigen::Matrix<double, 3, 3> R3 = Eigen::Matrix<double, 3, 3>::Zero();
    R3(0, 1) = R3(1, 2) = R3(2, 0) = 1;

    Eigen::Matrix<double, 3, 3> R4 = Eigen::Matrix<double, 3, 3>::Identity();
    R4(0, 0) = cos(init_vehicle_o);     R4(0, 1) = sin(init_vehicle_o);
    R4(1, 0) = -sin(init_vehicle_o);    R4(1, 1) = cos(init_vehicle_o);

    Eigen::Matrix<double, 3, 3> T_map2ecef_SLOW = (R4*R3*R2*R1).inverse();


    /*
     * result I want to find: sensor position with respect to ecef
     */
    Eigen::Matrix<double, 3, 1> sensor_p_ecef;//sensor position with respect to ecef coordinate system

    sensor_p_ecef = T_map2ecef_SLOW * sensor_p_map + init_vehicle_p;
    std::cout << "\tsensor_p_ecef_SLOW:\t" << sensor_p_ecef.transpose() << std::endl;




    Eigen::Matrix<double, 3, 3> T1 = Eigen::Matrix<double, 3, 3>::Identity();
    T1(0, 0) = cos(lon);        T1(0, 1) = -sin(lon);
    T1(1, 0) = sin(lon);        T1(1, 1) = cos(lon);

    Eigen::Matrix<double, 3, 3> T2 = Eigen::Matrix<double, 3, 3>::Identity();
    T2(0, 0) = cos(lat);        T2(0, 2) = -sin(lat);
    T2(2, 0) = sin(lat);        T2(2, 2) = cos(lat);

    Eigen::Matrix<double, 3, 3> T3 = Eigen::Matrix<double, 3, 3>::Zero();
    T3(0, 2) = T3(1, 0) = T3(2, 1) = 1;

    Eigen::Matrix<double, 3, 3> T4 = Eigen::Matrix<double, 3, 3>::Identity();
    T4(0, 0) = cos(init_vehicle_o);     T4(0, 1) = -sin(init_vehicle_o);
    T4(1, 0) = sin(init_vehicle_o);     T4(1, 1) = cos(init_vehicle_o);

    Eigen::Matrix<double, 3, 3> T_map2ecef = T1*T2*T3*T4;


    /*
     * result I want to find: sensor position with respect to ecef
     */

    sensor_p_ecef = T_map2ecef * sensor_p_map + init_vehicle_p;
    std::cout << "\tsensor_p_ecef:\t\t" << sensor_p_ecef.transpose() << std::endl;

    std::cout << "------------------------------------\n";

//
//
//    std::cout << "R1:\n" << R1 << std::endl;
//    std::cout << "R1 inversa:\n" << R1.inverse() << std::endl;
//    std::cout << "------------------------------------\n";
//
//    std::cout << "R2:\n" << R2 << std::endl;
//    std::cout << "R2 inversa:\n" << R2.inverse() << std::endl;
//    std::cout << "------------------------------------\n";
//
//    std::cout << "R3:\n" << R3 << std::endl;
//    std::cout << "R3 inversa:\n" << R3.inverse() << std::endl;
//    std::cout << "------------------------------------\n";
//
//    std::cout << "R4:\n" << R4 << std::endl;
//    std::cout << "R4 inversa:\n" << R4.inverse() << std::endl;
//    std::cout << "------------------------------------\n";
}
