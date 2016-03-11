//
// Created by ptirindelli on 10/03/16.
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

    Eigen::Matrix<double, 3, 1> _map_p(4789389, 176941, 4194510);
    Eigen::Matrix<double, 1, 1> _map_o; _map_o(0) = 30*M_PI/180;
    Eigen::Matrix<double, 2, 1> _vehicle_p(10, 20);
    Eigen::Matrix<double, 1, 1> _vehicle_o;  _vehicle_o(0) = 90*M_PI/180;
    Eigen::Matrix<double, 4, 1> sensor_p_base(3, 4, 5, 1);

    std::cout << "----------- originali -----------\n";
    std::cout << "map_p: " << _map_p[0] << ", " << _map_p[1] << ", " << _map_p[2] << std::endl;
    std::cout << "map_o: " << _map_o * 180 / M_PI << " [printed in degree]" << std::endl;
    std::cout << "vehicle_p_map: " << _vehicle_p[0] << ", " << _vehicle_p[1] << std::endl;
    std::cout << "vehicle_o_map: " << _vehicle_o[0] * 180 / M_PI << " [printed in degree]" << std::endl;
    std::cout << "sensor_p_base: " << sensor_p_base[0] << ", " << sensor_p_base[1] << ", " << sensor_p_base[2] << std::endl;
    std::cout << "---------------------------------\n";


    /*
     * Base-to-map transform matrix
     */
    Eigen::Matrix<double, 4, 4> T_base2map = Eigen::Matrix<double, 4, 4>::Identity();
    T_base2map(0, 0) = double(cos(_vehicle_o[0]));
    T_base2map(0, 1) = double(-sin(_vehicle_o[0]));
    T_base2map(1, 0) = double(sin(_vehicle_o[0]));
    T_base2map(1, 1) = double(cos(_vehicle_o[0]));
    T_base2map(0, 3) = double(_vehicle_p[0]);
    T_base2map(1, 3) = double(_vehicle_p[1]);

    std::cout << "T_base2map:\n" << T_base2map << std::endl << std::endl;

    // sensor position with respect to map frame
    Eigen::Matrix<double, 4, 1> sensor_p_map = T_base2map * sensor_p_base;


    std::cout << "!!! sensor_p_map: " << sensor_p_map.transpose() << std::endl;



    /*
     * _map_p from ECEF to LLA (math from https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf )
     */
    // WGS84 ellipsoid constants
    double a = double(6378137); // earth's radius
    double e = double(8.1819190842622e-2); // eccentricity
    double asq = a * a;
    double esq = e * e;
    double b = double(sqrt(asq * (double(1) - esq)));
    double bsq = double(b * b);
    double ep = double(sqrt((asq - bsq) / bsq));
    double p = double(sqrt(_map_p[0] * _map_p[0] + _map_p[1] * _map_p[1]));
    double th = double(atan2(a * _map_p[2], b * p));
    double lon = double(atan2(_map_p[1], _map_p[0]));
    double lat = double(atan2((_map_p[2] + ep * ep * b * pow(sin(th), 3)), (p - esq * a * pow(cos(th), 3))));

    // Module probably not needed. maybe only for printing reason
    //while (lon <= double(M_PI))        lon += double(2 * M_PI);
    //while (lon >  double(M_PI))        lon -= double(2 * M_PI);

    std::cout << "_map_p: " << _map_p[0] << ", " << _map_p[1] << ", " << _map_p[2] << std::endl;
    std::cout << "_map_p LLA: " << lat << ", " << lon /*<< ", " << alt*/ << std::endl;
    std::cout << "_map_p LLA degrees: " << lat * double(180 / M_PI) << ", " << lon * double(180 / M_PI) /*<< ", " << alt*/ << std::endl;

    /*
     * map-to-ECEF transform matrix
     * made by the product of the next 4 matrixes
     */
    Eigen::Matrix<double, 4, 4> T_lon2ecef = Eigen::Matrix<double, 4, 4>::Identity();
    T_lon2ecef(0, 0) = double(cos(lon));
    T_lon2ecef(0, 1) = double(-sin(lon));
    T_lon2ecef(1, 0) = double(sin(lon));
    T_lon2ecef(1, 1) = double(cos(lon));
    std::cout << "T_lon2ecef\n" << T_lon2ecef << std::endl << std::endl;

    Eigen::Matrix<double, 4, 4> T_lat2lon = Eigen::Matrix<double, 4, 4>::Identity();
    T_lat2lon(0, 0) = double(cos(lat));
    T_lat2lon(0, 2) = double(-sin(lat));
    T_lat2lon(2, 0) = double(sin(lat));
    T_lat2lon(2, 2) = double(cos(lat));
    std::cout << "T_lat2lon\n" << T_lat2lon << std::endl << std::endl;

    Eigen::Matrix<double, 4, 4> T_aux2lat = Eigen::Matrix<double, 4, 4>::Identity();
    T_aux2lat(0, 3) = double(_map_p[0]);
    T_aux2lat(1, 3) = double(_map_p[1]);
    T_aux2lat(2, 3) = double(_map_p[2]);
    std::cout << "T_aux2lat\n" << T_aux2lat << std::endl << std::endl;

    Eigen::Matrix<double, 4, 4> T_enu2aux = Eigen::Matrix<double, 4, 4>::Zero();
    T_enu2aux(0, 2) = T_enu2aux(1, 0) = T_enu2aux(2, 1) = T_enu2aux(3, 3) = double(1);
    std::cout << "T_enu2aux\n" << T_enu2aux << std::endl << std::endl;

    Eigen::Matrix<double, 4, 4> T_map2eno = Eigen::Matrix<double, 4, 4>::Identity();
    T_map2eno(0, 0) = double(cos(_map_o[0]));
    T_map2eno(0, 1) = double(-sin(_map_o[0]));
    T_map2eno(1, 0) = double(sin(_map_o[0]));
    T_map2eno(1, 1) = double(cos(_map_o[0]));
    std::cout << "T_map2eno\n" << T_map2eno << std::endl << std::endl;

//    Eigen::Matrix<double, 4, 4> T_map2ecef = T_lon2ecef * T_lat2lon * T_aux2lat * T_enu2aux * T_map2eno;
    Eigen::Matrix<double, 4, 4> T_map2ecef = T_aux2lat * T_lon2ecef * T_lat2lon * T_enu2aux * T_map2eno;
    std::cout << "---------T_map2ecef\n" << T_map2ecef << std::endl << std::endl;


    //sensor position with respect to ecef coordinate system
    Eigen::Matrix<double, 4, 1> sensor_p_ecef =  T_map2ecef * sensor_p_map;
    std::cout << "!!! sensor_p_ecef: " << sensor_p_ecef[0] << ", " << sensor_p_ecef[1] << ", " << sensor_p_ecef[2] << std::endl;

}
