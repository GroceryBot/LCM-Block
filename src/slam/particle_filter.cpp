#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>

ParticleFilter::ParticleFilter(int numParticles)
    : kNumParticles_(numParticles)
{
  assert(kNumParticles_ > 1);
  proposal.resize(kNumParticles_);
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t &pose)
{
  // proposal(kNumParticles_);
  ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
  //uniformly assigned weight
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.005);
  std::normal_distribution<double> theta_distribution(0.0, 0.011);

  std::cout << pose.x << " " << pose.y << " " << pose.theta << std::endl;
  double w = 1.0 / kNumParticles_;
  //randomly initialize posterior_
  for (int i = 0; i < kNumParticles_; i++)
  {
    particle_t randParticle;
    pose_xyt_t randPose;
    //Initialization around the start pose with normal error
    randPose.x = pose.x + distribution(generator);
    randPose.y = pose.y + distribution(generator);
    randPose.theta = pose.theta + theta_distribution(generator);
    randPose.utime = pose.utime;
    randParticle.pose = randPose;
    randParticle.weight = w;
    // std::cout<<"Rand pose "<<i<<" "<<randPose.x<<" "<<randPose.y<<" "<<randPose.theta<<" "<<randParticle.weight<<std::endl;
    proposal[i] = randParticle;
  }
  // std::cout<<"Initialization done. Posterior size: "<< posterior_.size()<<std::endl;
}

pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t &odometry,
                                        const lidar_t &laser,
                                        const OccupancyGrid &map)
{
  // Only update the particles if motion was detected. If the robot didn't move, then
  // obviously don't do anything.
  // proposal.clear();
  bool hasRobotMoved = actionModel_.updateAction(odometry);
  if (hasRobotMoved) modulo_num++;
  if (hasRobotMoved && modulo_num % 2)
  {
    modulo_num = 0;
    resamplePosteriorDistribution();
    computeProposalDistribution();
    computeNormalizedPosterior(laser, map);
    posteriorPose_ = estimatePosteriorPose();
  }
  posteriorPose_.utime = odometry.utime;

  return posteriorPose_;
}

pose_xyt_t ParticleFilter::poseEstimate(void) const
{
  return posteriorPose_;
}

particles_t ParticleFilter::particles(void) const
{
  particles_t particles;
  particles.num_particles = proposal.size();
  particles.particles = proposal;
  return particles;
}

void ParticleFilter::resamplePosteriorDistribution(void)
{
  //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.0035);
  std::normal_distribution<double> distribution_theta(0.0, 0.07);

  // std::vector<particle_t> prior;
  // prior.reserve(kNumParticles_);
  std::vector<double> weight(kNumParticles_);
  assert(!proposal.empty());
  // if (proposal.empty())
  //   return proposal;
  weight[0] = proposal[0].weight;

  //calculate cumulative weight distribution
  for (unsigned int i = 1; i < proposal.size(); i++)
  {
    weight[i] = weight[i - 1] + proposal[i].weight;
  }

  std::vector<particle_t> new_thing;
  new_thing.reserve(kNumParticles_);
  //draw M particles from previous poterior distribution
  for (int i = 0; i < kNumParticles_; i++)
  {
    double random_weight = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    std::vector<double>::iterator up;
    up = std::upper_bound(weight.begin(), weight.end(), random_weight);
    particle_t top = proposal[std::distance(weight.begin(), up)];
    top.pose.x += distribution(generator);
    top.pose.y += distribution(generator);
    top.pose.theta += distribution_theta(generator);
    new_thing.push_back(top);
  }
  proposal = new_thing;
  // return proposal;
}

void ParticleFilter::computeProposalDistribution()
{
  //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
  // std::vector<particle_t> proposal(kNumParticles_);
  for (unsigned int i = 0; i < proposal.size(); i++)
  {
    proposal[i] = actionModel_.applyAction(proposal[i]);
  }
}

void ParticleFilter::computeNormalizedPosterior(const lidar_t &laser,
                                                                   const OccupancyGrid &map)
{
  /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
  ///////////       particles in the proposal distribution
  double s = 0.0f;

  for (unsigned int i = 0; i < proposal.size(); i++)
  {
    proposal[i].weight = sensorModel_.likelihood(proposal[i], laser, map);
    s += proposal[i].weight;
  }
  //normalize particle weights (make them add up to one)
  for (unsigned int i = 0; i < proposal.size(); i++)
  {
    proposal[i].weight /= s;
    //if (posterior[i].weight>0.1){
    //std::cout<<"Hight weight particle: "<<i<<" weight: "<<posterior[i].weight<<posterior[i].pose.x<<' '<<posterior[i].pose.y<<std::endl;
    //}
    // std::cout << "likelihood " << posterior[i].weight << std::endl;
  }
  sensorModel_.init = false;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose()
{
  //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
  pose_xyt_t pose;
  float x = 0;
  float y = 0;
  float theta = 0;
  // std::cout << "Posterior size: " << posterior.size() << std::endl;
  float max_weight = -1;
  float min_weight = 1000;
  for (unsigned int i = 0; i < proposal.size(); i++)
  {
    if (proposal[i].weight > max_weight)
    {
      max_weight = proposal[i].weight;
      // x=posterior[i].pose.x;
      // y=posterior[i].pose.y;
      // theta=posterior[i].pose.theta;
    }
    if (proposal[i].weight < min_weight)
    {
      min_weight = proposal[i].weight;
    }
  }
  // pose.x = x;
  // pose.y = y;
  // pose.theta = theta;
  //return pose;
  // if (max_weight < 0.2)
  // return pose;
  float top_percent = min_weight + ((max_weight - min_weight) * 0.9);

  float s = 0.0f;
  for (unsigned int i = 0; i < proposal.size(); i++)
  {
    if (proposal[i].weight > top_percent) {
      s += proposal[i].weight;
    }
  }

  x = 0;
  y = 0;
  theta = 0;
  for (unsigned int i = 0; i < proposal.size(); i++)
  {
    if (proposal[i].weight > top_percent)
    {
      if (std::abs(proposal[i].pose.x) < 5 && std::abs(proposal[i].pose.y) < 5)
      {
        x += proposal[i].pose.x * (proposal[i].weight / s);
        y += proposal[i].pose.y * (proposal[i].weight / s);
        theta += proposal[i].pose.theta * (proposal[i].weight / s);
      }
  // std::cout<<"Estimated pose: "<< x<<" "<< y<<" "<< theta<<std::endl;
    }
  }
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  // std::cout<<"Estimated pose: "<< pose.x<<" "<< pose.y<<" "<< pose.theta<<std::endl;
  return pose;
}
