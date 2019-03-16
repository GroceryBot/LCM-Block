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
#include <unistd.h>
#include <signal.h>
#include <fstream>

#define MAX_ANGULAR_SPEED 1.0f
#define MIN_ANGULAR_SPEED 0.3f
#define MAX_TRANS_SPEED 0.75f
#define MIN_TRANS_SPEED 0.05f
#define PI 3.14159265f
#define MAX_ANGLE_TOLERANCE 0.025f

double MAX_TRANS_TOLERANCE = 2.5;
double THETA_P = 0.65;
double THETA_I = 0.0;
double THETA_D = 0.06;
double X_P = 0.13;
double X_I = 0.00;
double X_D = 0.06;
std::ofstream csv;

class PIDController
{
  public:
    PIDController() {}

    PIDController(float _gainP, float _gainI, float _gainD, pose_xyt_t _set_point, pose_xyt_t current_pose)
    {
        /* Construct a PID controller. */
        gainP = _gainP;
        // gainI uses microseconds, so to use reasonable numbers we divide.
        gainI = _gainI;
        gainD = _gainD;

        last_pose = current_pose;
        integrated_error = 0;
        set_point = _set_point;
    }

    void update_set_point(pose_xyt_t _set_point)
    {
        /* Change the PID controller's set point. */
        set_point = _set_point;
    }

    double getNextAngularVelocity(pose_xyt_t current_pose)
    {
        /* Get the drive direction in the form of a pose.
         *
         * Though a pose is a convenient representation, the return value doesn't
         * represent a robot pose, but instead the arc the robot should travel on
         * (determined by normalized x, y, and theta), and the speed at which it
         * should drive (determined by the magnitude of the vector).
         */

        // Calculate dx, dy, dt, d0
        double current_theta_err = angle_diff(set_point.theta, current_pose.theta);
        double dt = (current_pose.utime - last_pose.utime) / 1000000.0;

        double last_theta_err = angle_diff(set_point.theta, last_pose.theta);
        double diff_err = current_theta_err - last_theta_err;
        // update member variables
        last_pose = current_pose;

        integrated_error += current_theta_err * dt;

        double output;
        output = current_theta_err * gainP;
        output += integrated_error * gainI;
        output += diff_err / dt * gainD;
        csv << output << "," << current_theta_err << "," << integrated_error << "," << diff_err << std::endl;
        return output;
    }

    double getNextTransVelocity(pose_xyt_t current_pose)
    {
        /* Get the drive direction in the form of a pose.
         *
         * Though a pose is a convenient representation, the return value doesn't
         * represent a robot pose, but instead the arc the robot should travel on
         * (determined by normalized x, y, and theta), and the speed at which it
         * should drive (determined by the magnitude of the vector).
         */

        // Calculate dx, dy, dt, d0
        double current_distance_err = distance(current_pose, set_point);
        double dt = (current_pose.utime - last_pose.utime) / 1000000.0;

        double last_distance_err = distance(last_pose, set_point);
        double diff_err = current_distance_err - last_distance_err;
        // update member variables
        last_pose = current_pose;

        integrated_error += current_distance_err * dt;

        double output;
        output = current_distance_err * gainP;
        output += integrated_error * gainI;
        output += diff_err / dt * gainD;

        if (current_distance_err < 0.1)
        {
            output = current_distance_err;
        }

        return output;
    }

    double distance(pose_xyt_t start, pose_xyt_t end)
    {
        return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2) * 1.0);
    }

  public:
    double integrated_error;

  private:
    float gainP;
    float gainI;
    float gainD;
    pose_xyt_t last_pose;
    pose_xyt_t set_point;
};

float clamp_speed_trans(float speed)
{
    if (speed > 0 && speed < MIN_TRANS_SPEED)
    {
        return MIN_TRANS_SPEED;
    }
    if (speed < 0 && speed > -MIN_TRANS_SPEED)
    {
        return -MIN_TRANS_SPEED;
    }
    if (speed < -MAX_TRANS_SPEED)
    {
        return -MAX_TRANS_SPEED;
    }
    else if (speed > MAX_TRANS_SPEED)
    {
        return MAX_TRANS_SPEED;
    }

    return speed;
}

float clamp_speed_angular(float speed)
{
    if (speed > 0 && speed < MIN_ANGULAR_SPEED)
    {
        return MIN_ANGULAR_SPEED;
    }
    if (speed < 0 && speed > -MIN_ANGULAR_SPEED)
    {
        return -MIN_ANGULAR_SPEED;
    }
    if (speed < -MAX_ANGULAR_SPEED)
    {
        return -MAX_ANGULAR_SPEED;
    }
    else if (speed > MAX_ANGULAR_SPEED)
    {
        return MAX_ANGULAR_SPEED;
    }

    return speed;
}

class MotionController
{
  public:
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM *instance) : lcmInstance(instance)
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

        pose_xyt_t pose;
        pose.utime = pose.x = pose.y = pose.theta = 0;
        odomTrace_.addPose(pose);
        targets_.push_back(pose);

        theta_pid = PIDController(THETA_P, THETA_I, THETA_D, pose, pose);
        x_pid = PIDController(X_P, X_I, X_D, pose, pose);
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
        mbot_motor_command_t cmd;
        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;
        cmd.utime = now();

        // Check if reaches target according to odometry
        if (haveReachedTarget())
        {
            last_trans_speed = 0;
            std::cout << "--------------------\n";
            std::cout << "TARGET REACHED\n";
            bool haveTarget = assignNextTarget();
            if (!haveTarget)
            {
                std::cout << "--------------------\n";
                std::cout << "COMPLETED PATH!\n";
            }
        }

        //////////////// Actual PID control codes goes in here ////////////////
        else if (!targets_.empty() && !odomTrace_.empty())
        {
            // Use feedback based on heading error for line-of-sight vector pointing to the target.
            // Convert odometry to the global coordinates
            pose_xyt_t target = targets_.back();
            pose_xyt_t pose = currentPose();

            // calculate target heading
            target.theta = atan2((target.y - pose.y), (target.x - pose.x));
            double angular_error = angle_diff_abs(target.theta, pose.theta);

            if (state_ == TURN) // if the car is in turning mode (no translational velocity)
            {
                last_trans_speed = 0;
                cmd.trans_v = 0;
                if (angular_error > MAX_ANGLE_TOLERANCE) // turning is linear feedback control
                {
                    theta_pid.update_set_point(target);
                    double angular_velocity_pid = theta_pid.getNextAngularVelocity(pose);

                    // std::cout << "ACC ANGULAR: " << std::abs((1.0 * angular_velocity_pid) - (1.0 * last_angular_speed)) << std::endl;
                    // if (std::abs((1.0 * angular_velocity_pid) - (1.0 * last_angular_speed)) > 0.05)
                    // {
                    //     if (angular_velocity_pid > last_angular_speed)
                    //     {
                    //         angular_velocity_pid = last_angular_speed + 0.05;
                    //     }
                    //     else
                    //     {
                    //         angular_velocity_pid = last_angular_speed - 0.05;
                    //     }
                    // }
                    cmd.angular_v = clamp_speed_angular(angular_velocity_pid);
                }
                else
                {
                    theta_pid.integrated_error = 0;
                    state_ = DRIVE;
                }
            }
            else if (state_ == DRIVE) // Use PID to drive to the target once approximately pointed in the correct direction
            {
                cmd.angular_v = 0.0f;
                // Use PIDs to get magnitude of output

                if (angular_error > MAX_ANGLE_TOLERANCE * 8) // turning is linear feedback control
                {
                    x_pid.integrated_error = 0;
                    last_trans_speed = 0;
                    state_ = TURN;
                }
                else
                {
                    // theta_pid.update_set_point(target);
                    // double angular_velocity_pid = theta_pid.getNextAngularVelocity(pose);
                    // cmd.angular_v = clamp_speed(angular_velocity_pid);
                    double trans_speed = x_pid.getNextTransVelocity(pose);
                    if (trans_speed < 0)
                    {
                        std::cout << "NEGATIVE VALUE" << std::endl;
                        std::cout << last_trans_speed << std::endl;
                        std::cout << trans_speed << std::endl;
                        std::cout << "_______________" << std::endl;
                    }
                    if (std::abs((1.0 * trans_speed) - (1.0 * last_trans_speed)) > 0.0005)
                    {
                        std::cout << "ACC : " << std::abs((1.0 * trans_speed) - (1.0 * last_trans_speed)) << std::endl;
                        if (trans_speed > 0 && last_trans_speed < 0)
                        {
                            trans_speed = 0.0005;
                        }
                        else if (trans_speed < 0 && last_trans_speed > 0)
                        {
                            trans_speed = 0.0005;
                        }
                        else if (trans_speed > last_trans_speed)
                        {
                            trans_speed = last_trans_speed + 0.0005;
                        }
                        else
                        {
                            trans_speed = last_trans_speed - 0.0005;
                        }
                    }
                    cmd.trans_v = clamp_speed_trans(trans_speed);
                    std::cout << "OUTPUT: " << cmd.trans_v << std::endl;
                }
            }
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
            // std::cout << "Moving to  : " << target.x << " " << target.y << " " << target.theta << std::endl;
            // std::cout << "Moving from: " << pose.x << " " << pose.y << " " << pose.theta << std::endl;
            // std::cout << "trans_v: " << cmd.trans_v << " angular_v: " << cmd.angular_v << std::endl;
            // std::string state_string = state_ == DRIVE ? "DRIVE" : "TURN";
            // std::cout << "in state: " << state_string << std::endl;
            // std::cout << "---------" << std::endl;
        }
        // if targets or odomTrace is not initialized, command the robot to do nothing
        last_trans_speed = cmd.trans_v;
        last_angular_speed = cmd.angular_v;
        return cmd;
    }

    float distance(pose_xyt_t start, pose_xyt_t end)
    {
        return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2) * 1.0);
    }

    bool timesync_initialized() { return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer *buf, const std::string &channel, const timestamp_t *timesync)
    {
        timesync_initialized_ = true;
        time_offset = timesync->utime - utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer *buf, const std::string &channel, const robot_path_t *path)
    {
        // clear old path
        targets_.clear();

        // assigns targets to vector of poses corresponding to path
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end());

        std::cout << "received new path at time: " << path->utime << "\n";
        for (auto pose : targets_)
        {
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }
        std::cout << "\n";

        confirm.utime = now();
        confirm.creation_time = path->utime;
        confirm.channel = channel;
        state_ = TURN;

        //confirm that the path was received
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer *buf, const std::string &channel, const odometry_t *odometry)
    {
        /////// TODO: Implement your handler for new odometry data ////////////////////
        // done
        pose_xyt_t pose;
        pose.utime = odometry->utime;
        pose.x = odometry->x;
        pose.y = odometry->y;
        pose.theta = odometry->theta;
        odomTrace_.addPose(pose);
    }

    void handlePose(const lcm::ReceiveBuffer *buf, const std::string &channel, const pose_xyt_t *pose)
    {
        computeOdometryOffset(*pose);
    }

  private:
    enum State
    {
        TURN,
        DRIVE,
    };

    pose_xyt_t odomToGlobalFrame_; // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace odomTrace_;          // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;
    int64_t end_time_ = 0;

    double last_trans_speed = 0;
    double last_angular_speed = 0;
    // State of the motion controller
    State state_ = TURN;

    // One PID controller for distance, one for theta
    PIDController theta_pid;
    PIDController x_pid;

    bool timesync_initialized_;
    int64_t time_offset;

    message_received_t confirm;
    lcm::LCM *lcmInstance;

    int64_t now()
    {
        return utime_now() + time_offset;
    }

    bool haveReachedTarget(void)
    {
        const float kPosTolerance = 0.025f;
        const float kFinalPosTolerance = 0.025f;

        //tolerance for intermediate waypoints can be more lenient
        float tolerance = (targets_.size() == 1) ? kFinalPosTolerance : kPosTolerance;

        // There's no target, so we're there by default.
        if (targets_.empty())
        {
            return false;
        }
        // If there's no odometry, then we're nowhere, so we couldn't be at a target
        if (odomTrace_.empty())
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
        if (!targets_.empty())
        {
            targets_.pop_back();
        }

        theta_pid.update_set_point(targets_.back());
        theta_pid.integrated_error = 0;
        x_pid.update_set_point(targets_.back());
        x_pid.integrated_error = 0;
        // x_pid.update_set_point(targets_.back());
        state_ = TURN;
        return !targets_.empty();
    }

    void computeOdometryOffset(const pose_xyt_t &globalPose)
    {
        /////// TODO: Implement your handler for new pose data ////////////////////
        // odomToGlobalFrame_.x = 0;
        // odomToGlobalFrame_.y = 0;
        // odomToGlobalFrame_.theta = 0;
        //pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime + time_offset);
        // double deltaTheta = angle_diff(odomAtTime.theta, globalPose.theta);
        // double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        // double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
        //odomToGlobalFrame_.x = globalPose.x - odomAtTime.x;
        //odomToGlobalFrame_.y = globalPose.y - odomAtTime.y;
        //odomToGlobalFrame_.theta = -odomAtTime.theta + globalPose.theta;

        pose_xyt_t pose_t = odomTrace_.poseAt(globalPose.utime+time_offset);
        odomToGlobalFrame_.x = -pose_t.x + globalPose.x;
        odomToGlobalFrame_.y = -pose_t.y + globalPose.y;
        odomToGlobalFrame_.theta = -pose_t.theta + globalPose.theta;

        //std::cout << "ODOMETRY OFFSET" << std::endl;
        //std::cout << "X: " << odomToGlobalFrame_.x << " Y: " << odomToGlobalFrame_.y << " TH: " << odomToGlobalFrame_.theta << std::endl;
        //std::cout << "_____________" << std::endl;
    }

    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = odomPose.x + odomToGlobalFrame_.x;
        pose.y = odomPose.y + odomToGlobalFrame_.y;
        pose.theta = odomPose.theta; //+ odomToGlobalFrame_.theta;
        // pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) + odomToGlobalFrame_.x;
        // pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta)) + odomToGlobalFrame_.y;
        // pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
        pose.utime = now();
        return pose;
    }
};

int main(int argc, char **argv)
{

    csv.open("pid.csv");
    if (argc == 7)
    {
        X_P = atof(argv[1]);
        X_I = atof(argv[2]);
        X_D = atof(argv[3]);
        std::cout << "X_P: " << X_P << std::endl;
        std::cout << "X_I: " << X_I << std::endl;
        std::cout << "X_D: " << X_D << std::endl;
        THETA_P = atof(argv[4]);
        THETA_I = atof(argv[5]);
        THETA_D = atof(argv[6]);
        std::cout << "THETA_P: " << THETA_P << std::endl;
        std::cout << "THETA_I: " << THETA_I << std::endl;
        std::cout << "THETA_D: " << THETA_D << std::endl;
    }
    lcm::LCM lcmInstance(MULTICAST_URL);

    MotionController controller(&lcmInstance);
    lcmInstance.subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, &controller);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, &controller);
    lcmInstance.subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, &controller);
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, &controller);

    signal(SIGINT, exit);

    while (true)
    {
        lcmInstance.handleTimeout(50); // update at 20Hz minimum

        if (controller.timesync_initialized())
        {
            mbot_motor_command_t cmd = controller.updateCommand();

            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        }
    }

    return 0;
}
