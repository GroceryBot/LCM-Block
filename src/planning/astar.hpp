#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <common/point.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <limits>


class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs

    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path

    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};


class Node
{
public:
  Point<int> n;
  const Node *p;
  float f_score;
  float g_score;

  Node(Point<int> n, const Node *p, float g_score):
  n(Point<int>(n.x, n.y)), p(p), f_score(std::numeric_limits<float>::infinity()), g_score(g_score)
  {};
};


struct Compare
{
    bool operator()(const Node& lhs, const Node& rhs)
    {
      return lhs.f_score > rhs.f_score;
    }
};


struct hash
{
    size_t operator()(Point<int> const & x) const
    {
        return (
            (51 + std::hash<int>()(x.x)) * 51
            + std::hash<int>()(x.y)
        );
    }
};



  /**
  * search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
  *
  * \param    start           Starting pose of the robot
  * \param    goal            Desired goal pose of the robot
  * \param    distances       Distance to the nearest obstacle for each cell in the grid
  * \param    params          Parameters specifying the behavior of the A* search
  * \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
  *   pose is returned, per the robot_path_t specification.
  */
robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);


//Test if the destination is reached.
bool isDestinationReached(const Node &n, const Node &dest);


//Calculate f_score = distance from start + heuristic distance

float calculateHscore(const Node &n, const Node &dest,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);


#endif // PLANNING_ASTAR_HPP
