#include "nav_data.h"
#include "raw_gps_utils.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    
    //Welcome message
    cout << endl << " ========= test ===========" << endl << endl;

    NavData nav("ciao", rawgpsutils::ScalarT(42), rawgpsutils::ScalarT(100000), rawgpsutils::ScalarT(654654), rawgpsutils::ScalarT(456456));

    cout << nav.getSatId() << endl;

    return 0;
}


