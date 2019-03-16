#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
*
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal
* distribution for the particle filter.
*/
class ActionModel
{
public:

    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);

    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const pose_xyt_t& odometry);

    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);

private:

    ////////// TODO: Add private member variables needed for you implementation ///////////////////

    //TODO: these member variables should be further tuned
  const float val = 0.5;
  const float alpha1 = val;
  const float alpha2 = val;
  const float alpha3 = 1;
  const float alpha4 = 1;
  const float alpha5 = val;
  const float alpha6 = val;
  float rot1_var;
  float trans_var;
  float rot2_var;

  float del_rot1;
  float del_trans;
  float del_rot2;
  std::vector<pose_xyt_t> lastPose_;
};

#endif // SLAM_ACTION_MODEL_HPP
