#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

void SensorModel::generateOccupied(const OccupancyGrid& map) {
    occupied.clear();
    for (int i = 0; i < map.widthInCells(); ++i) {
        for (int j = 0; j < map.heightInCells(); ++j) {
            if (map.logOdds(i, j) > 126) {
                occupied.push_back({i, j});
            }
        }
    }
}

float distance(float x1, float y1, float x2, float y2) {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1));
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    if (!init) {
        generateOccupied(map);
        init = true;
    }
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose);
    float q = 0.0;
    // float total = 0.0;
    for (unsigned int i = 0; i < ml_scan.size(); ++i) {
        float theta = ml_scan[i].theta;
        float range = ml_scan[i].range;
        int x = (ml_scan[i].origin.x + range*cos(theta))*20+100;
        int y = (ml_scan[i].origin.y + range*sin(theta))*20+100;
        //std::cout<<ml_scan[i].origin.x<<" "<<ml_scan[i].origin.y<<" "<<x<<" "<<y<<std::endl;
        //TODO: should be tuned for better result
        double min_distance = 1000;
        // printf("BBBBBBBB=%d\n", occupied.size());
        for (unsigned int k = 0; k < occupied.size(); ++k) {
            float dist = distance(x, y, occupied[k].first, occupied[k].second);
            if (dist < min_distance) {
                min_distance = dist;
            }
            if (min_distance == 0) break;
        }
        if (min_distance <= 0.1) {
            min_distance = 0.5;
        }
        q += 1.0/(min_distance);

        // if(map.logOdds(x,y)>30){
        //   hits += 1.0;
        // }
        // else if (map.logOdds(x + 1,y)>30 || map.logOdds(x - 1,y)>30 || map.logOdds(x,y + 1)>30 ||map.logOdds(x,y - 1)>30 || map.logOdds(x+1,y+1)>30
        //     || map.logOdds(x+1,y-1)>30 ||map.logOdds(x-1,y+1)>30 || map.logOdds(x -1,y-1)>30) {
        //     hits += 0.5;
        // }
        // total += 1.0;
    }
    return q;
}
