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
        float x = (ml_scan[i].origin.x + calculateX(ml_scan[i].range, ml_scan[i].theta)) * 20 + 100;
        float y = (ml_scan[i].origin.y + calculateY(ml_scan[i].range, ml_scan[i].theta)) * 20 + 100;
        int val = map.logOdds(floor(x), floor(y)) + kHitOdds_;
        if (val > 127) val = 127;
        map.setLogOdds(floor(x), floor(y), val);

    	for (float dist_ = 0; dist_ < ml_scan[i].range; dist_ += 0.05) {
    		float x_intermediate = (ml_scan[i].origin.x + calculateX(dist_, ml_scan[i].theta)) * 20 + 100;
    		float y_intermediate = (ml_scan[i].origin.y + calculateY(dist_, ml_scan[i].theta)) * 20 + 100;
            val = map.logOdds(floor(x_intermediate), floor(y_intermediate)) - kMissOdds_;
    		//printf("val %d\n", val);
    		//printf("kMissOdds %d\n", kMissOdds_);
    		if (val < -127) val = -127;
            if (!(floor(x) == floor(x_intermediate) && floor(y) == floor(y_intermediate)))
    		  map.setLogOdds(floor(x_intermediate), floor(y_intermediate), val);
    	}
    }
    last_pose = pose;

}
