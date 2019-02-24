#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////

    int64_t last_time = 0;
    const float alpha1 = 0.01;
    const float alpha2 = 0.01;
    const float alpha3 = 0.01;
    const float alpha4 = 0.01;
    const float alpha5 = 0.01;
    const float alpha6 = 0.01;
    float vhat_var = 0;
    float omegahat_var = 0;
    float gammahat_var = 0;
    float v = 0;
    float w = 0;
    int64_t dt = 0;
}

float sample_normal_dist(const float bsquared)
{
  float retval = 0;
  for(int i = 0; i < 12; ++i) {
    float LO = -sqrt(bsquared);
    float HIGH = sqrt(bsquared);
    retval += LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HIGH - LO)));
  }
  retval *= 0.5；
  return retval;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    if (last_time==0){
      last_time = odometry.utime;
      std::cout<<"No last odometry"<<std::endl;
      return false;
    }
    v = odometry.fwd_velocity;
    w = odometry.ang_velocity;
    //float x = odometry.x;
    //float y = odometry.y;
    //float theta = odometry.theta;
     dt = odometry.utime - last_time;

    vhat_var = alpha1*v*v + alpha2*w*w;
    omegahat_var = alpha3*v*v + alpha4*w*w;
    gammahat_var = alpha5*v*v + alpha6*w*w;

    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    last_time = odometry.utime;
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    new_sample.parent_pose = sample.pose;
    pose_xyt_t pose = sample.pose;

    pose_xyt_t new_pose;
    float vhat = v + sample_normal_dist(vhat_var);
    float omegahat = w + sample_normal_dist(omegahat_var);
    float gammahat = sample_normal_dist(gammahat_var);
    new_pose.x = pose.x - vhat/omegahat *sin(pose.theta) + vhat/omegahat * sin(pose.theta + omegahat * dt);
    new_pose.y = pose.y + vhat/omegahat * cos(pose.theta) - vhat/omegahat *cos(pose.theta + omegahat * dt);
    new_pose.theta = pose.theta + omegahat*dt + gammahat*dt;
    new_sample.pose = new_pose;

    return new_sample;
}
