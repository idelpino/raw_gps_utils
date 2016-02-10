#include "satellites_obs.h"
//#include "raw_gps_utils.h"
//#include <iostream>

using namespace std;

int main(int argc, char** argv)
{

    cout << endl << " ========= test obs data ===========" << endl;

    string name = "ciao OBS";
    rawgpsutils::SatellitesObs obs;

    obs.time_gps_wnc_ = 3;
    obs.time_gps_tow_ = 4;

    rawgpsutils::PrMeasurement m(5,
                                 rawgpsutils::ScalarT(50),
                                 rawgpsutils::ScalarT(500),
                                 rawgpsutils::ScalarT(500),
                                 rawgpsutils::ScalarT(500),
                                 rawgpsutils::ScalarT(150),
                                 rawgpsutils::ScalarT(150),
                                 rawgpsutils::ScalarT(150));

    obs.measurements_.push_back(m);


    rawgpsutils::PrMeasurement m1(6,
                                  rawgpsutils::ScalarT(50),
                                  m.sat_velocity_,
                                  m.sat_position_);

    obs.addPrMeasurement(m1);

    obs.addPrMeasurement(rawgpsutils::PrMeasurement(7,
                                                    rawgpsutils::ScalarT(50),
                                                    rawgpsutils::ScalarT(500),
                                                    rawgpsutils::ScalarT(500),
                                                    rawgpsutils::ScalarT(500)));

    obs.addPrMeasurement(rawgpsutils::PrMeasurement(8,
                                                    rawgpsutils::ScalarT(50),
                                                    m.sat_velocity_));

    cout << obs.toString();

    cout << endl << " ========= END ===========" << endl;

    return 0;
}


