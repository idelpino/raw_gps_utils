//
// Created by ptirindelli on 18/12/15.
//

#ifndef RAW_GPS_UTILS_OBS_DATA_H
#define RAW_GPS_UTILS_OBS_DATA_H

// raw_gps_utils includes
#include "time_stamp.h"
#include "raw_gps_utils.h"

// Std includes
#include <iostream>

namespace rawgpsutils
{
    class PrMeasurement
    {
    public:
        PrMeasurement(int _sat_id,
                      ScalarT _pseudorange,
                      const Eigen::Vector3s &_sat_position,
                      const Eigen::Vector3s &_sat_velocity = Eigen::Vector3s(0, 0, 0));

        PrMeasurement(int _sat_id,
                      ScalarT _pseudorange,
                      ScalarT _x, ScalarT _y, ScalarT _z,
                      ScalarT _v_x = 0, ScalarT _v_y = 0, ScalarT _v_z = 0);



    public:
        int sat_id_;
        rawgpsutils::ScalarT pseudorange_;
        Eigen::Vector3s sat_position_;
        Eigen::Vector3s sat_velocity_;
    };

    class SatellitesObs
    {
    public:
        void addPrMeasurement(PrMeasurement m);
    public:
//        TODO fare un getter per il tempo
//        e che trasformi da tempo gps a tempo nel loro formato
//        usando gpstk magari

        rawgpsutils::ScalarT time_ros_;
        rawgpsutils::ScalarT time_gps_wnc_;
        rawgpsutils::ScalarT time_gps_tow_;
        std::vector<PrMeasurement> measurements_;

        std::string toString();

    };

}

#endif //RAW_GPS_UTILS_OBS_DATA_H
