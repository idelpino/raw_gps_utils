//
// Created by ptirindelli on 18/12/15.
//

#ifndef RAW_GPS_UTILS_OBS_DATA_H
#define RAW_GPS_UTILS_OBS_DATA_H


// Std includes
#include <iostream>
#include <eigen3/Eigen/Geometry>

namespace rawgpsutils
{
    /*
     * Class containing a single pseudorange measurement
     * and info about the satellites
     */
    class PrMeasurement
    {
    public:
        PrMeasurement(int _sat_id,
                      double _pseudorange,
                      const Eigen::Vector3d &_sat_position,
                      const Eigen::Vector3d &_sat_velocity = Eigen::Vector3d(0, 0, 0));

        PrMeasurement(int _sat_id,
                      double _pseudorange,
                      double _x, double _y, double _z,
                      double _v_x = 0, double _v_y = 0, double _v_z = 0);

    public:
        int sat_id_;
        double pseudorange_;
        Eigen::Vector3d sat_position_;
        Eigen::Vector3d sat_velocity_;
    };


    /*
     * Class containing a single pseudorange measurement
     * and info about the satellites
     */
    class SatellitesObs
    {
    public:
        void addPrMeasurement(PrMeasurement m);
    public:
//        TODO fare un getter per il tempo
//        e che trasformi da tempo gps a tempo nel loro formato
//        usando gpstk magari

        double time_ros_sec_;
        double time_ros_nsec_;
        double time_gps_wnc_;
        double time_gps_tow_;
        std::vector<PrMeasurement> measurements_;

        std::string toString();

    };

}

#endif //RAW_GPS_UTILS_OBS_DATA_H
