#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

void generateOccupied(const OccupancyGrid& map, std::vector<std::pair<int, int>>& occupied) {
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
    // if (!init) {
    //     generateOccupied(map, occupied);
    //     init = true;
    // }
    //     std::vector<std::pair<int, int>> occupied;

    // for (int i = 0; i < map.widthInCells(); ++i) {
    //     for (int j = 0; j < map.heightInCells(); ++j) {
    //         if (map.logOdds(i, j) > 126) {
    //             occupied.push_back({i, j});
    //         }
    //     }
    // }
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose);
    // float q = 0.0;
    float hits = 0.0;
    float total = 0.0;
    for (unsigned int i = 0; i < ml_scan.size(); ++i) {
        float theta = ml_scan[i].theta;
        float range = ml_scan[i].range;
        int x = (ml_scan[i].origin.x + range*cos(theta))*20+100;
        int y = (ml_scan[i].origin.y + range*sin(theta))*20+100;
        // std::cout<<ml_scan[i].origin.x<<" "<<ml_scan[i].origin.y<<" "<<x<<" "<<y<<std::endl;
    //     TODO: should be tuned for better result
    //     double min_distance = 1000;

    //     for (unsigned int k = 0; k < occupied.size(); ++k) {
    //         float dist = distance(x, occupied[k].first, y, occupied[k].second);
    //         if (dist < min_distance) {
    //             min_distance = dist;
    //         }
    //         if (min_distance == 0) break;
    //     }
    //     if (min_distance == 0) {
    //         min_distance = 0.1;
    //     }
    //     q += 1.0/(min_distance);
    // }
        // printf("BBBBBBBB=%f\n", min_distance);


        if(map.logOdds(x,y)>60){
          hits += 1.0;
        }
        else if (map.logOdds(x + 1,y)>60 || map.logOdds(x - 1,y)>60 || map.logOdds(x,y + 1)>60 ||map.logOdds(x,y - 1)>60 || map.logOdds(x+1,y+1)>60
            || map.logOdds(x+1,y-1)>60 ||map.logOdds(x-1,y+1)>60 || map.logOdds(x -1,y-1)>60) {
            hits += 0.7;
        }
        else if (map.logOdds(x + 2,y)>20 || map.logOdds(x + 2,y+1)>20 || map.logOdds(x + 2,y-1)>20 || map.logOdds(x - 2,y)>20 || map.logOdds(x -2,y+1)>20 || map.logOdds(x -2,y-1)>20 || map.logOdds(x,y + 2)>20 ||map.logOdds(x+1,y + 2)>20 || map.logOdds(x-1,y + 2)>20 
                    || map.logOdds(x,y - 2)>20 || map.logOdds(x+1,y - 2)>20 || map.logOdds(x-1,y - 2)>20 || map.logOdds(x+2,y+2)>20 || map.logOdds(x+2,y-2)>20 ||map.logOdds(x-2,y+2)>20 || map.logOdds(x -2,y-2)>20) {
                    hits += 0.4;
        }

        total += 1.0;
    }
    return hits/total;


        // printf("AAAAAAA=%f\n", q);
    // return q;

}
