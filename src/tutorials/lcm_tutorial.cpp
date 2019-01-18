// lcm cpp functionality
#include <lcm/lcm-cpp.hpp>

// Defines for lcm in this project
#include <common/lcm_config.h>
// The message type for oled generated from lcmtypes/oled_message_t.lcm
#include <lcmtypes/oled_message_t.hpp>
// Code for producing timestamps
#include <common/timestamp.h>
// The message type for odometry
#include <lcmtypes/odometry_t.hpp>

#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

#include <unistd.h>

class OdomHandler
{
  public:
    // hold odometry
    odometry_t odom_;
    // hold channel name
    std::string channel_;
    // A callback for odometry messages
    void handle(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* msg)
    {
        // Copy
        odom_ = *msg;
        channel_ = channel;
    }
};

int main(int argc, char ** argv)
{
    // Initilize lcm
    lcm::LCM lcm(MULTICAST_URL);
    if(!lcm.good())
        return 1;

    // Define an oled message
    // we can look up the definition in lcmtypes/oled_message_t.lcm
    oled_message_t msg;
    // Fill in the timestamp
    msg.utime = utime_now();
    // Fill in the two lines of the oled
    msg.line1 = "Hello";
    msg.line2 = "world";

    while(1) {
        std::cout << "LCM Tutorial\n";
        lcm.publish(OLED_CHAN, &msg);
        // Publish at 1 second intervals
        usleep(1000000);
    }
}
