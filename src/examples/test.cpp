#include "nav_data.h"
#include "obs_data.h"
//#include "raw_gps_utils.h"
//#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    cout << endl << " ========= test nav data ===========" << endl;

    rawgpsutils::NavData nav("ciao NAV", TimeStamp(42), rawgpsutils::ScalarT(100000), rawgpsutils::ScalarT(654654), rawgpsutils::ScalarT(456456));

    cout << nav.getSatId() << endl;
    cout << "Timestamp: " << nav.getTimestamp().get() << endl;



    cout << endl << " ========= test obs data ===========" << endl;

    string name = "ciao OBS";
    rawgpsutils::ObsData obs(name, TimeStamp(43), rawgpsutils::ScalarT(100000));
    cout << obs.getSatId() << endl;
    cout << "Timestamp: " << obs.getTimestamp().get() << endl;

    cout << endl << " ========= END ===========" << endl;

    return 0;
}


