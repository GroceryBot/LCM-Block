#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <limits>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    const float  max_lidar = 8;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
  // implemented according to page 172 of probabilistic robotics
    double q = 1.0;
    MovingLaserScan ml_scan(scan, sample.parent_pose, sample.pose);
    for (int i = 0; i < ml_scan.size(); ++i) {
      // if (ml_scan[i].range < 5){ turns out we don't need this, talked to some other people in the class
        float theta = ml_scan[i].theta;
        float range = ml_scan[i].range;
        float x = ml_scan[i].origin.x + range*cos(theta);
        float y = ml_scan[i].origin.y + range*sin(theta);
        float dist = findDistance(x, y, map);
        double Z_HIT_CONST = 1.0; //needs to be tuned
        double STDEV_CONST = 1.0; // needs to be tuned
        q *= Z_HIT_CONST * prob_normal_dist(dist, STDEV_CONST);
        // }
    }
    return q;
}

double findDistance(float x, float y, const OccupancyGrid& map) {
  // this can probably be heavily optimized by keeping a set of occupied cells
  double min_distance = std::numeric_limits<float>::max();
  for (int i = 0; i < map.widthInCells(); ++i) {
    for (int j = 0; j < map.heightInCells(); ++j) {
      if (map.logOdds(i, j) > 125) { //occupied
        double temp_distance = distance(x * map.cellsPerMeter() + 100, y * map.cellsPerMeter() + 100, i, j);
        if (temp_distance < min_distance)
        {
          min_distance = temp_distance;
        }
      }
    }
  }
  return min_distance;
}

double prob_normal_dist(double value, double stdev) {
  // used formula from https://stattrek.com/probability-distributions/normal.aspx
  const double pi = 3.14159265358979323846;
  const double e = 2.71828;
  const double mean = 0.0;
  double exp = (-pow((value - mean), 2)) / (2 * pow(stdev, 2));
  return (1.0/(stdev * sqrt(2.0*pi))) * pow(e, exp);
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
