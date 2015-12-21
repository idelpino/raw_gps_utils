//
// Created by ptirindelli on 18/12/15.
//

#ifndef RAW_GPS_UTILS_NAVDATA_H
#define RAW_GPS_UTILS_NAVDATA_H

// raw_gps_utils includes
#include "time_stamp.h"
#include "raw_gps_utils.h"

// Std includes
#include <iostream>

class NavData {
public:
    NavData(const std::string &_sat_id, const TimeStamp &_timestamp, const rawgpsutils::ScalarT &_pseudorange, const rawgpsutils::ScalarT &_param1, const rawgpsutils::ScalarT &_param2);
    
    const std::string & getSatId() const;
    
    const TimeStamp & getTimestamp() const;
    
    rawgpsutils::ScalarT getPseudorange() const;
    
    rawgpsutils::ScalarT getParam1() const;
    
    rawgpsutils::ScalarT getParam2() const;

protected:
    std::string sat_id_;
    TimeStamp timestamp_;
    rawgpsutils::ScalarT pseudorange_;
    rawgpsutils::ScalarT param1_;
    rawgpsutils::ScalarT param2_;
};


#endif //RAW_GPS_UTILS_NAVDATA_H
