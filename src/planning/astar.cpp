#include "queue"
#include "unordered_set"
#include "stack"
#include "algorithm"

#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/point.hpp>
#include <planning/Node.hpp>

using std::unordered_set;
using std::priority_queue;
robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);
    path.path_length = path.path.size();

    

    return path;
}
