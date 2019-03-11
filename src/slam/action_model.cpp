#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void):
rot1_var(0.07), trans_var(0.07), rot2_var(0.07),del_rot1(0.07), del_trans(0.07), del_rot2(0.07)
{}

float sample_normal_dist(const float bsquared)
{
  float retval = 0;
  for(int i = 0; i < 12; ++i) {
    float LO = -sqrt(bsquared);
    float HIGH = sqrt(bsquared);
    retval += LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HIGH - LO)));
  }
  retval *= 0.5;
  return retval;
}

bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
  ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    //Save the pose at the first step and return false
    if (lastPose_.empty()){
      lastPose_.push_back(odometry);
      //std::cout<<"No last odometry"<<std::endl;
      return false;
    }

    //Robot didn't move, return flase
    if (lastPose_[0].x == odometry.x && lastPose_[0].y == odometry.y && lastPose_[0].theta == odometry.theta){
      return false;
    }

    float x = lastPose_[0].x;
    float y = lastPose_[0].y;
    float theta = lastPose_[0].theta;

    float xprime = odometry.x;
    float yprime  = odometry.y;
    float thetaprime  = odometry.theta;

    //Calculate the distribution paramters
    del_rot1 = atan2(yprime-y, xprime-x) - theta;
    del_trans = sqrt((x-xprime)*(x-xprime) + (y-yprime)*(y-yprime));
    del_rot2 = thetaprime - theta - del_rot1;

    rot1_var = alpha1*del_rot1*del_rot1 + alpha2*del_trans*del_trans;
    trans_var = alpha3*del_trans*del_trans + alpha4*del_rot1*del_rot1 + alpha4*del_rot2*del_rot2;
    rot2_var = alpha1*del_rot2*del_rot2 + alpha2*del_trans*del_trans;
    lastPose_[0] = odometry;
    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    new_sample.parent_pose = sample.pose;
    pose_xyt_t pose = sample.pose;
    pose_xyt_t new_pose;

    float del_rot1_hat = del_rot1 - sample_normal_dist(rot1_var);
    float del_rot2_hat = del_rot2 - sample_normal_dist(rot2_var);
    float del_trans_hat = del_trans - sample_normal_dist(trans_var);

    new_pose.x = pose.x + del_trans_hat*cos(pose.theta + del_rot1_hat);
    new_pose.y = pose.y + del_trans_hat*sin(pose.theta + del_rot1_hat);
    new_pose.theta = pose.theta + del_rot1_hat + del_rot2_hat;
    new_sample.pose = new_pose;

    return new_sample;
}
