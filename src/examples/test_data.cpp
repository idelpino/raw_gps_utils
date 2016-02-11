#include "satellites_obs.h"

using namespace std;

int main(int argc, char** argv)
{

    cout << endl << " ========= test obs data ===========" << endl;

    string name = "ciao OBS";
    rawgpsutils::SatellitesObs obs;

    obs.time_gps_wnc_ = 3;
    obs.time_gps_tow_ = 4;

    obs.time_ros_sec_ = 356;
    obs.time_ros_nsec_ = 24546544;

    rawgpsutils::PrMeasurement m(5,50, 500, 500, 500, 150, 150, 150);
    obs.measurements_.push_back(m);


    rawgpsutils::PrMeasurement m1(6, 50, m.sat_velocity_, m.sat_position_);
    obs.addPrMeasurement(m1);

    obs.addPrMeasurement(rawgpsutils::PrMeasurement(7, 50, 500, 500, 500));

    obs.addPrMeasurement(rawgpsutils::PrMeasurement(8, 50, m.sat_velocity_));



    cout << obs.toString();
    cout << endl << " ========= END ===========" << endl;

    return 0;
}


