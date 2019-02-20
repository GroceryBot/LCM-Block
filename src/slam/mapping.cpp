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
    	last_pose = pose;
    	started = true;
    	return;
    }
    if (pose.x != last_pose.x || pose.y != last_pose.y || pose.theta != last_pose.theta) {
    	map.reset();
    }
    MovingLaserScan ml_scan(scan, last_pose, pose);
    for (int i = 0; i < ml_scan.size(); ++i) {
    	float x = (ml_scan[i].origin.x + calculateX(ml_scan[i].range, ml_scan[i].theta)) * 20 + 100;
    	float y = (ml_scan[i].origin.y + calculateY(ml_scan[i].range, ml_scan[i].theta)) * 20 + 100;
    	map.setLogOdds(x, y, 127);

    }
    last_pose = pose;

}
