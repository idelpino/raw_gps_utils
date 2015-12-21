#include "nav_data.h"

NavData::NavData(const std::string &_sat_id, const TimeStamp &_timestamp, const rawgpsutils::ScalarT &_pseudorange, const rawgpsutils::ScalarT &_param1, const rawgpsutils::ScalarT &_param2)
        : sat_id_(_sat_id), timestamp_(_timestamp), pseudorange_(_pseudorange), param1_(_param1), param2_(_param2)
{
    std::cout << "Received navigation data for satellite " << sat_id_ << std::endl;
}

const std::string &NavData::getSatId() const
{
    return sat_id_;
}

const TimeStamp &NavData::getTimestamp() const
{
    return timestamp_;
}

rawgpsutils::ScalarT NavData::getPseudorange() const
{
    return pseudorange_;
}

rawgpsutils::ScalarT NavData::getParam1() const
{
    return param1_;
}

rawgpsutils::ScalarT NavData::getParam2() const
{
    return param2_;
}