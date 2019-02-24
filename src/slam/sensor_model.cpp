#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    const float  max_lidar = 8;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    float q = 1.0;
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose);
    for (int i = 0; i < ml_scan.size(); ++i) {
      if (ml_scan[i].range < map.){
        float theta = ml_scan[i].theta;
        float range = ml_scan[i].range;
        float x = ml_scan[i].origin.x + range*cos(theta);
        float y = mu_scan[i].origin.y + range*sin(theta);
        float dist = 
      }
    }

    return 1.0;
}
