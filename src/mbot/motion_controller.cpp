#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include <math.h>

#define PI 3.14159265358979323846
float clamp_speed(float speed)
{
    if(speed < -1.0f)
    {
        return -1.0f;
    }
    else if(speed > 1.0f)
    {
        return 1.0f;
    }

    return speed;
}


class MotionController
{
public:

    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance) : lcmInstance(instance)
    {
        ////////// TODO: Initialize your controller state //////////////

        // Initially, there's no offset between odometry and the global state
        odomToGlobalFrame_.x = 0.0f;
        odomToGlobalFrame_.y = 0.0f;
        odomToGlobalFrame_.theta = 0.0f;

	    time_offset = 0;
	    timesync_initialized_ = false;

        confirm.utime = 0;
        confirm.creation_time = 0;
        confirm.channel = "";
    }

    /**
    * updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    *
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void)
    {
        //////////// TODO: Implement your feedback controller here. //////////////////////
        // Use the data from odomTrace_ to update the next pose, odomTrace_ is a vector of type pose_xyt_t

        //initialization of motor command
        mbot_motor_command_t cmd;
        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;
        cmd.utime = now();
        if(targets_.empty()){
            return cmd;
        }

        if(haveReachedTarget())
        {
            std::cout << "TARGET REACHED\n";
            bool haveTarget = assignNextTarget();

            if(!haveTarget)
            {
                std::cout << "COMPLETED PATH!\n";
                return cmd;
            }
        }


        // Use feedback based on heading error for line-of-sight vector pointing to the target.
        pose_xyt_t target = targets_.back();

        //Make target longer to reach
        //target.x *= 1.1;
        //target.y *= 1.1;
        // Convert odometry to the global coordinates
        pose_xyt_t pose = currentPose();

        target.theta = atan2((pose.y-target.y), (pose.x-target.x));
        if (target.theta>0){
            target.theta -= M_PI;
        }
        else{
             target.theta += M_PI;
        }
        std::cout<<"atan theta: "<<target.theta<<std::endl;


        if(new_path_received_){
            new_path_received_ = false;
            end_time_ = now()+100000000; //timeout period
        }
        if(end_time_ > now()){
            std::cout<<"POSE: x, y, theta: "<<pose.x<<" "<<pose.y<<" "<<pose.theta<<std::endl;
            std::cout<<"TARGET: x, y, theta: "<<target.x<<" "<<target.y<<" "<<target.theta<<std::endl;
    	    float ang_tolerance = (turn90) ? 0.05 : 0.1;
    	    std::cout<<"Angle tolerance: "<<ang_tolerance<<std::endl;

            if(std::abs(target.theta - pose.theta)<ang_tolerance){
                state_=State::DRIVE;
                std::cout<<"State: drive.\n";
                turn90 = false;
            }
            else{
                state_=State::TURN;
                std::cout<<"State: turn.\n";
            }

            if(state_== State::TURN){
                //TODO: maybe change trans_v not 0 when turning
                cmd.trans_v = 0.08f;
                cmd.angular_v = clamp_speed(turn_pid(target, pose));
                std::cout<<"AV command: "<<cmd.angular_v<<std::endl;
            }
            else if(state_ == State::DRIVE){
                cmd.angular_v = 0.0f;
                cmd.trans_v = clamp_speed(drive_pid(target, pose));
                std::cout<<"TV command: "<<cmd.trans_v<<std::endl;
            }
        }
        else{
        std::cout<<"TIME OUT!!!\n";
        }
        return cmd;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync){
        timesync_initialized_ = true;
        time_offset = timesync->utime-utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        /////// TODO: Implement your handler for new paths here ////////////////////
        targets_ = path->path;
        //targets_.erase(targets_.begin());
        //std::reverse(targets_.begin(), targets_.end());
    	std::cout << "received new path at time: " << path->utime << "\n";
    	for(auto pose : targets_){
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}std::cout << "\n";
        new_path_received_ = true;

        confirm.utime = now();
        confirm.creation_time = path->utime;
        confirm.channel = channel;

        //confirm that the path was received
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        /////// TODO: Implement your handler for new odometry data ////////////////////
        pose_xyt_t pose;
        pose.x = odometry->x;
        pose.y = odometry->y;
        pose.theta = odometry->theta;
        pose.utime = odometry->utime;
        odomTrace_.addPose(pose);
        currentFV = odometry->fwd_velocity;
        currentAV = odometry->ang_velocity;
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        //std::cout<<"SLAM data: "<<pose->x<<" "<<pose->y<<" "<<pose->theta<<std::endl;
        //std::cout<<"Time diff: "<<now()-pose->utime<<std::endl;
        computeOdometryOffset(*pose);
    }

private:

    enum State
    {
        TURN,
        DRIVE,
    };

    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;

    // State of the motion controller
    State state_ = State::TURN;

    int64_t time_offset;

    bool timesync_initialized_;

    message_received_t confirm;
    lcm::LCM * lcmInstance;

    bool new_path_received_ = false;
    int64_t end_time_ = 0;

    const float Kp_drive = 0.03f;
    const float Ki_drive = 0.0005f;
    const float Kd_drive = 0.001f;

    const float Kp_angular = 0.2f;
    const float Ki_angular = 0.001f;
    const float Kd_angular = 0.001f;
    bool turn90 = false;

    float currentFV = 0.0;
    float currentAV = 0.0;
    float Ui_turn = 0.0;
    float Ui_drive = 0.0;
    int64_t now(){
        return utime_now() + time_offset;
    }


    float turn_pid(const pose_xyt_t& target, const pose_xyt_t& current){
        float Up = Kp_angular*clamp_radians(target.theta - current.theta);
        std::cout<<"Current AV: "<<currentAV<<std::endl;
        float Ud = Kd_angular*(-currentAV);
        Ui_turn += Ki_angular*clamp_radians(target.theta - current.theta);
        return Up+Ud+Ui_turn;
    }


    float drive_pid(const pose_xyt_t& target, const pose_xyt_t& current){
        float e = sqrt((target.x - current.x)*(target.x - current.x) + (target.y - current.y)*(target.y - current.y));
        float Up = Kp_drive*(e);
        float Ud = Kd_drive*(currentFV);
        Ui_drive += Ki_drive*e;
        return Up+Ud+Ui_drive;
    }

    bool haveReachedTarget(void)
    {
        const float kPosTolerance = 0.25f;
	    const float kFinalPosTolerance = 0.15f;

        //tolerance for intermediate waypoints can be more lenient
    	float tolerance = (targets_.size() == 1) ? kFinalPosTolerance : kPosTolerance;

        // There's no target, so we're there by default.
        if(targets_.empty())
        {
            return false;
        }
        // If there's no odometry, then we're nowhere, so we couldn't be at a target
        if(odomTrace_.empty())
        {
            return false;
        }

        pose_xyt_t target = targets_.back();
        pose_xyt_t pose = currentPose();

        float xError = std::abs(target.x - pose.x);
        float yError = std::abs(target.y - pose.y);
        return (xError < tolerance) && (yError < tolerance);
    }

    bool assignNextTarget(void)
    {
        // If there was a target, remove it
        if(!targets_.empty())
        {
            targets_.pop_back();
        }
        // TODO: Reset all error and state terms when switching to a new target
        state_ = State::TURN;
        Ui_turn = 0.0;
        Ui_drive = 0.0;
        turn90 = true;
        return !targets_.empty();
    }

    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        /////// TODO: Implement your handler for new pose data ////////////////////
        pose_xyt_t pose_t = odomTrace_.poseAt(globalPose.utime+time_offset);
        odomToGlobalFrame_.x = -pose_t.x + globalPose.x;
        odomToGlobalFrame_.y = -pose_t.y + globalPose.y;
        odomToGlobalFrame_.theta = -pose_t.theta + globalPose.theta;
    }

    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = odomPose.x+odomToGlobalFrame_.x;
        pose.y = odomPose.y+odomToGlobalFrame_.y;
        pose.theta = odomPose.theta+odomToGlobalFrame_.theta;
        pose.utime = now();
        // TODO: Implement transform from odom frame to slam frame
        return pose;
    }

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

};


int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);

    MotionController controller(&lcmInstance);
    lcmInstance.subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, &controller);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, &controller);
    lcmInstance.subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, &controller);
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, &controller);

    signal(SIGINT, exit);

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            mbot_motor_command_t cmd = controller.updateCommand();

            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}
    }

    return 0;
}
