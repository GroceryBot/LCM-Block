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
        std::cout << "got something\n";
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

    // Define a string stream to do float to string conversion
    std::stringstream ss;

    // Create an odom handler
    OdomHandler odom_handle;
    // Subscribe to all topics matching regx .*ODOMETRY
    // When new messages are recieved the handleOdometry callback will be called
    lcm.subscribe(".*ODOMETRY", &OdomHandler::handle, &odom_handle);

    // loop as long as lcm handle doesn't return an error
    while (lcm.handle() == 0) {
        std::cout << "LCM Tutorial\n";
        // Fill in the oled msg
        msg.utime = utime_now();
        msg.line1 = odom_handle.channel_;
        ss << "x: " << odom_handle.odom_.x << "  y:" << odom_handle.odom_.y << "  theta: " << odom_handle.odom_.theta;
        msg.line2 = ss.str();
        std::cout << msg.line1 << std::endl << msg.line2 << std::endl;
        lcm.publish(OLED_CHAN, &msg);

        // Clear ss
        ss.clear();
        ss.str(std::string());
        // Publish at 100 hz
        usleep(10000);
    }
}
