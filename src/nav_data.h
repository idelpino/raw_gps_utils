//
// Created by ptirindelli on 18/12/15.
//

#ifndef WOLF_NAVDATA_H
#define WOLF_NAVDATA_H

#include "raw_gps_utils.h"
#include <iostream>

class NavData {
public:
    NavData(const std::string &_sat_id, const rawgpsutils::ScalarT &_timestamp, const rawgpsutils::ScalarT &_pseudorange, const rawgpsutils::ScalarT &_param1, const rawgpsutils::ScalarT &_param2);
    
    const std::string & getSatId() const;
    
    const rawgpsutils::ScalarT & getTimestamp() const;
    
    rawgpsutils::ScalarT getPseudorange() const;
    
    rawgpsutils::ScalarT getParam1() const;
    
    rawgpsutils::ScalarT getParam2() const;

protected:
    std::string sat_id_;
    rawgpsutils::ScalarT timestamp_;
    rawgpsutils::ScalarT pseudorange_;
    rawgpsutils::ScalarT param1_;
    rawgpsutils::ScalarT param2_;
};


#endif //WOLF_NAVDATA_H
