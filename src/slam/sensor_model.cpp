#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose);
    float hits = 0.0;
    float total = 0.0;
    for (unsigned int i = 0; i < ml_scan.size(); ++i) {
        float theta = ml_scan[i].theta;
        float range = ml_scan[i].range;
        int x = (ml_scan[i].origin.x + range*cos(theta))*20+100;
        int y = (ml_scan[i].origin.y + range*sin(theta))*20+100;
        //std::cout<<ml_scan[i].origin.x<<" "<<ml_scan[i].origin.y<<" "<<x<<" "<<y<<std::endl;
        //TODO: should be tuned for better result
        if(map.logOdds(x,y)>20){
          hits += 1.0;
        }
        else if (map.logOdds(x + 1,y)>20 || map.logOdds(x - 1,y)>20 || map.logOdds(x,y + 1)>20 ||map.logOdds(x,y - 1)>20 || map.logOdds(x+1,y+1)>20
            || map.logOdds(x+1,y-1)>20 ||map.logOdds(x-1,y+1)>20 || map.logOdds(x -1,y-1)>20) {
            hits += 0.7;
        }
        else if (map.logOdds(x + 2,y)>20 || map.logOdds(x + 2,y+1)>20 || map.logOdds(x + 2,y-1)>20 || map.logOdds(x - 2,y)>20 || map.logOdds(x -2,y+1)>20 || map.logOdds(x -2,y-1)>20 || map.logOdds(x,y + 2)>20 ||map.logOdds(x+1,y + 2)>20 || map.logOdds(x-1,y + 2)>20 
            || map.logOdds(x,y - 2)>20 || map.logOdds(x+1,y - 2)>20 || map.logOdds(x-1,y - 2)>20 || map.logOdds(x+2,y+2)>20 || map.logOdds(x+2,y-2)>20 ||map.logOdds(x-2,y+2)>20 || map.logOdds(x -2,y-2)>20) {
            hits += 0.4;
        }
        total += 1.0;
    }
    printf("BBBBBBBB=%f\n", hits/total);
    return hits/total;
}
