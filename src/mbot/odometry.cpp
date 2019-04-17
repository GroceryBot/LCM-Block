/*******************************************************************************
* odometry.cpp
*
* TODO: Implement these functions to add odometry and dead rekoning
*
*******************************************************************************/

#include <mbot/mbot_channels.h>
#include <mbot/mbot_defs.h>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/mbot_imu_t.hpp>
#include <lcmtypes/mbot_encoder_t.hpp>
#include <lcmtypes/reset_odometry_t.hpp>
#include <common/lcm_config.h>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <signal.h>
#include <iostream>

#define PI 3.14159265358979323846

class Odometry
{
  private:
    lcm::LCM* lcm_instance_;
    float x_, y_, theta_, prev_imu_theta_, delta_imu_theta_;
    int64_t last_time_;


  public:
    /*******************************************************************************
    * initialize_odometry()
    *
    * TODO: initialize odometry
    *
    *******************************************************************************/
    Odometry(lcm::LCM* lcm_instance) : lcm_instance_(lcm_instance), x_(0), y_(0), theta_(0), delta_imu_theta_(0), last_time_(0) {}


    /*******************************************************************************
    * handleEncoders()
    *
    * TODO: calculate odometry from internal variables
    *       publish new odometry to lcm ODOMETRY_CHANNEL
    *
    *******************************************************************************/
    void handleIMU(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_imu_t* msg){
        delta_imu_theta_ =msg->tb_angles[2] - prev_imu_theta_;
        prev_imu_theta_ = msg->tb_angles[2];
        // std::cout << "0: " << msg->tb_angles[0] << std::endl;
        // std::cout << "1: " << msg->tb_angles[1] << std::endl;
        // std::cout << "2: " << msg->tb_angles[2] << std::endl;

        // std::cout << "IMU :" << delta_imu_theta_ << " : " << prev_imu_theta_ << std::endl;
    }

    void handleEncoders(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_encoder_t* msg){
        // Skip the first encoder reading
        if (last_time_ == 0)
        {
            last_time_ = msg->utime;
            return;
        }

        // Publish odometry msg
        double DistancePerCount = (PI * WHEEL_DIAMETER) / (ENCODER_RES * GEAR_RATIO);
        double LengthBetweenTwoWheels = WHEEL_BASE;

        //               num_tick / (48 tick/rev * gearratio)
        // rev / sec = ---------------------------------------
        //                      time difference (sec)
        // angular velocity = 2 * PI * rev / sec = rad / sec   (angular velocity of the wheel)

        float dleft = msg->left_delta * DistancePerCount;
        float dright = msg->right_delta * DistancePerCount;

        // [dx dy d0] = [.5 .5 .5; 0 0 1; -1/b 1/b 0] [dleft dright dslip]

        float dx = 0.5 * (dleft + dright);
        float dy = 0;
        float d0 = 1/LengthBetweenTwoWheels * (dright - dleft);
        // std::cout<<"d0 "<<d0<<std::endl;
        int64_t dt = msg->utime - last_time_;
        // if(std::abs(msg->left_delta - msg->right_delta) > 60){
        //     // std::cout << "\n d0 IMU : " << delta_imu_theta_ <<std::endl;
        //     d0 = delta_imu_theta_;
        // }
        if(std::abs(delta_imu_theta_) > 0.005){
            std::cout << "\n d0 IMU : " << delta_imu_theta_ <<std::endl;
            d0 = delta_imu_theta_;
        }

        theta_ += d0;
        // std::cout << "\n qwer Theta "<<theta_<<std::endl;
        x_ += dx * cos(theta_) - dy * sin(theta_);
        y_ += dx * sin(theta_) + dy * cos(theta_);

        odometry_t odom_msg;

        odom_msg.utime = msg->utime;
        odom_msg.x = x_;
        odom_msg.y = y_;

        odom_msg.left_velocity = dleft / dt;
        odom_msg.right_velocity = dright / dt;
        odom_msg.fwd_velocity = dx / dt;
        odom_msg.ang_velocity = d0 / dt;
        odom_msg.theta = theta_;
        lcm_instance_->publish(ODOMETRY_CHANNEL, &odom_msg);

        // printf("x: %f\ny: %f\ntheta: %f", x_, y_, theta_);
    }


    /*******************************************************************************
    * handleEncoders()
    *
    * TODO: calculate odometry from internal variables
    *       publish new odometry to lcm ODOMETRY_CHANNEL
    *
    *******************************************************************************/
    void handleOdometryReset(const lcm::ReceiveBuffer* buf, const std::string& channel, const reset_odometry_t* msg){
        reset(msg->x, msg->y, msg->theta);
    }


    void reset(float x = 0, float y = 0, float theta = 0) {
        x_ = x;
        y_ = y;
        theta_ = clamp_radians(theta);
    }


  private:
    /*******************************************************************************
    * clamp_radians()
    *******************************************************************************/
    float clamp_radians(float angle){

        if(angle < -PI)
        {
            for(; angle < -PI; angle += 2.0*PI);
        }
        else if(angle > PI)
        {
            for(; angle > PI; angle -= 2.0*PI);
        }

        return angle;
    }

    float angle_diff_radians(float angle1, float angle2){
        float diff = angle2 - angle1;
        while(diff < -PI) diff+=2.0*PI;
        while(diff > PI) diff-=2.0*PI;
        return diff;
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);

    Odometry odom(&lcmInstance);
    lcmInstance.subscribe(MBOT_ENCODERS_CHANNEL, &Odometry::handleEncoders, &odom);
    lcmInstance.subscribe(MBOT_IMU_CHANNEL, &Odometry::handleIMU, &odom);
    lcmInstance.subscribe(RESET_ODOMETRY_CHANNEL, &Odometry::handleOdometryReset, &odom);

    signal(SIGINT, exit);

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum


    }

    return 0;
}
