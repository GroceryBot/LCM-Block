#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <math.h>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}

float calculateX(float distance, float theta) {
	return distance * cos(theta);
}
float calculateY(float distance, float theta) {
	return distance * sin(theta);
}
void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!started) {
    	map.reset();
    	last_pose = pose;
    	started = true;
    	return;
    }
    MovingLaserScan ml_scan(scan, last_pose, pose);
    for (unsigned int i = 0; i < ml_scan.size(); ++i) {
        float x = (ml_scan[i].origin.x + calculateX(ml_scan[i].range, ml_scan[i].theta));
        float y = (ml_scan[i].origin.y + calculateY(ml_scan[i].range, ml_scan[i].theta));
        float error = -1.0;
        float dx = x - ml_scan[i].origin.x;
        float dy = y - ml_scan[i].origin.y;
        float d_error = abs(dy/dx);
        float cur_y = ml_scan[i].origin.y;
        for (float cur_x = ml_scan[i].origin.x; cur_x < x; cur_x += 0.05) {
            //UPDATE MAP START
            
            //UPDATE MAP END
            error = error + d_error;
            if (error > 0.0) {
                cur_y = y + 0.05;
                error -= 1;
            }
        }
    }
    last_pose = pose;
}
