#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    //uniformly assigned weight
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.01);
    
    std::cout<<pose.x<<" "<<pose.y<<" "<<pose.theta<<std::endl;
    double w =  1.0/kNumParticles_;
    //randomly initialize posterior_
    for (int i=0; i<kNumParticles_; i++){
      particle_t randParticle;
      pose_xyt_t randPose;
      //Initialization around the start pose with normal error 
      randPose.x = pose.x + distribution(generator); 
      randPose.y = pose.y + distribution(generator); 
      randPose.theta = pose.theta + distribution(generator); 
      randPose.utime = pose.utime;
      randParticle.pose = randPose;
      randParticle.weight = w;
      std::cout<<"Rand pose "<<i<<" "<<randPose.x<<" "<<randPose.y<<" "<<randPose.theta<<" "<<randParticle.weight<<std::endl;
      posterior_[i] = randParticle;
    }
    std::cout<<"Initialization done. Posterior size: "<< posterior_.size()<<std::endl;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
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
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    std::vector<particle_t> prior;
    std::vector<double> weight(kNumParticles_);
    weight[0] = 0.0;

    //calculate cumulative weight distribution
    for(unsigned int i=1; i<posterior_.size(); i++){
      weight[i]=weight[i-1]+posterior_[i-1].weight;
    }

    //draw M particles from previous poterior distribution
    for(int i=0; i<kNumParticles_; i++){
      double random_weight = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
      //binary search to pick particle -- TODO: this can be changed to better performance algorithm
      int l = 0;
      int r = kNumParticles_-1;

      while (l<=r){
        int m = l+(r-l)/2;
        if (m+1<kNumParticles_){
          printf("weight %f\n", weight[m]);
          if (weight[m] < random_weight && weight[m + 1] >= random_weight)
          {
            prior.push_back(posterior_[m]);
            break;
          }
          if (weight[m] < random_weight)
            l = m+1;
          else
            r = m-1;
        }
        else{
          prior.push_back(posterior_[m]);
          break;
        }
      }
    }
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal=prior;
    for(unsigned int i=0;i<prior.size(); i++){
      proposal[i] = actionModel_.applyAction(prior[i]);
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior=proposal;
    double s = 0.0f;

    for(unsigned int i=0; i<proposal.size();i++){
      posterior[i].weight = sensorModel_.likelihood(proposal[i],laser, map);
      s+=posterior[i].weight;
    }
    //normalize particle weights (make them add up to one)
    for(unsigned int i=0; i<proposal.size();i++){
      posterior[i].weight /=s;
      //if (posterior[i].weight>0.1){
        //std::cout<<"Hight weight particle: "<<i<<" weight: "<<posterior[i].weight<<posterior[i].pose.x<<' '<<posterior[i].pose.y<<std::endl;
      //}
      //std::cout<<"likelihood "<<posterior[i].weight<<std::endl;
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    float x = 0;
    float y = 0;
    float theta = 0;
    //std::cout<<"Posterior size: "<<posterior.size()<<std::endl;
    for(unsigned int i=0; i<posterior.size();i++){
      if (std::abs(posterior[i].pose.x) < 5 && std::abs(posterior[i].pose.y) < 5){
        x+=posterior[i].pose.x * posterior[i].weight;
        y+=posterior[i].pose.y * posterior[i].weight;
        theta+=posterior[i].pose.theta * posterior[i].weight;
      }
    }
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    // std::cout<<"Estimated pose: "<< pose.x<<" "<< pose.y<<" "<< pose.theta<<std::endl;
    return pose;
}
