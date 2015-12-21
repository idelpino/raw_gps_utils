//
// Created by ptirindelli on 18/12/15.
//

#include "obs_data.h"

ObsData::ObsData(const std::string &_sat_id, const TimeStamp &_timestamp, const rawgpsutils::ScalarT &_pseudorange)
        : sat_id_(_sat_id), timestamp_(_timestamp), pseudorange_(_pseudorange)
{
    //std::cout << "ObsData constructor: " << toString();
}

void ObsData::calculateSatPosition()
{
    sat_position_ = Eigen::Vector3s(50000+pseudorange_, 60000+pseudorange_, 70000+pseudorange_);
}

std::string ObsData::toString()
{
    std::ostringstream s;
    s << "SatID: " << sat_id_ << " -- " << timestamp_.get() << " -- " << pseudorange_ << " -- (" << sat_position_[0] << ", " << sat_position_[1] << ", " << sat_position_[2];
    return s.str();
}

const std::string &ObsData::getSatId() const
{
    return sat_id_;
}

const TimeStamp &ObsData::getTimestamp() const
{
    return timestamp_;
}

rawgpsutils::ScalarT ObsData::getPseudorange() const
{
    return pseudorange_;
}

Eigen::Vector3s ObsData::getSatPosition() const
{
    return sat_position_;
}

void ObsData::setSatPosition(const Eigen::Vector3s &sat_position_)
{
    ObsData::sat_position_ = sat_position_;
}