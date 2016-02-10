//
// Created by ptirindelli on 18/12/15.
//

#include "satellites_obs.h"

using namespace rawgpsutils;


std::string SatellitesObs::toString()
{
    std::ostringstream s;
    s << "Obs: " << measurements_.size() << " sats at time TODO:\n";


    for (int i = 0; i < measurements_.size(); ++i)
    {
    s << "\tSat" << measurements_[i].sat_id_ << " --pr" << measurements_[i].pseudorange_
      << " --pos(" << measurements_[i].sat_position_[0] << ", " << measurements_[i].sat_position_[1] << ", " << measurements_[i].sat_position_[2] << ")"
      << " --vel(" << measurements_[i].sat_velocity_[0] << ", " << measurements_[i].sat_velocity_[1] << ", " << measurements_[i].sat_velocity_[2] << ")"
      << std::endl;
    }

    return s.str();
}

PrMeasurement::PrMeasurement(int _sat_id,
                             ScalarT _pseudorange,
                             const Eigen::Vector3s &_sat_position,
                             const Eigen::Vector3s &_sat_velocity)
        : sat_id_(_sat_id),
          pseudorange_(_pseudorange),
          sat_position_(_sat_position),
          sat_velocity_(_sat_velocity)
{

}

PrMeasurement::PrMeasurement(int _sat_id,
                             ScalarT _pseudorange,
                             ScalarT _x, ScalarT _y, ScalarT _z,
                             ScalarT _v_x, ScalarT _v_y, ScalarT _v_z)
        : sat_id_(_sat_id),
          pseudorange_(_pseudorange)
{
    sat_position_[0] = _x;
    sat_position_[1] = _y;
    sat_position_[2] = _z;

    sat_velocity_[0] = _v_x;
    sat_velocity_[1] = _v_y;
    sat_velocity_[2] = _v_z;
}


void SatellitesObs::addPrMeasurement(PrMeasurement m)
{
    measurements_.push_back(m);
}
