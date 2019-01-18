// lcm cpp functionality
#include <lcm/lcm-cpp.hpp>

// Defines for lcm in this project
#include <common/lcm_config.h>

#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

#include <unistd.h>

int main(int argc, char ** argv)
{
    // Initilize lcm
    lcm::LCM lcm(MULTICAST_URL);
    if(!lcm.good())
        return 1;

    while(1) {
        std::cout << "LCM Tutorial\n";
        // Publish at 1 second intervals
        usleep(1000000);
    }
}
