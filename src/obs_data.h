//
// Created by ptirindelli on 18/12/15.
//

#ifndef RAW_GPS_UTILS_OBS_DATA_H
#define RAW_GPS_UTILS_OBS_DATA_H


#include "raw_gps_utils.h"


class ObsData
{
public:
    ObsData(const std::string &_sat_id, const rawgpsutils::ScalarT &_timestamp, rawgpsutils::ScalarT &_pseudorange);

    void calculateSatPosition();

    std::string toString();

    const std::string & getSatId() const;

    const rawgpsutils::ScalarT & getTimestamp() const;

    rawgpsutils::ScalarT getPseudorange() const;

    Eigen::Vector3s getSatPosition() const;

    void setSatPosition(const Eigen::Vector3s &sat_position_);

protected:
    std::string sat_id_;
    rawgpsutils::ScalarT timestamp_;
    rawgpsutils::ScalarT pseudorange_;
    Eigen::Vector3s sat_position_;

};


#endif //RAW_GPS_UTILS_OBS_DATA_H
