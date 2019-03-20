#include "queue"
#include "unordered_set"
#include "stack"
#include "algorithm"

#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/point.hpp>
#include <planning/Node.hpp>

using std::priority_queue;
using std::unordered_set;
robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid &distances,
                             const SearchParams &params)
{
  robot_path_t path;
  path.utime = start.utime;
  path.path_length = path.path.size();

  Node *startNode = new Node(Point<int>(start.x * 20 + 100, start.y * 20 + 100), nullptr, 0);
  startNode->f_score = h_score(startNode, goalNode, distances);

  Node *goalNode = new Node(Point<int>(goal.x * 20 + 100, goal.y * 20 + 100), nullptr, 0);

  unordered_set<Point> closedSet;
  set<Node *, Compare> openSet;
  // unordered_set<Node *, hash, equal> openSet_s;

  openSet.push(startNode);
  // openSet_s.insert(startNode->n);

  while (!openSet.empty())
  {
    Node *curr = openSet.begin(); //CHECK THIS
    if (*curr == *goalNode)
    {
      return reconstruct_path(curr, start.utime);
    }
    else
    {
      openSet.remove(openSet.begin());
      closedSet.insert(curr->n);
      vector<Node *> neighbors;
      int curr_x = curr->n.x;
      int curr_y = curr->n.y;
      neighbors.push_back(new Node(Point<int>(curr_x, curr_y + 1), curr, curr->g_score + 1);     // N
      neighbors.push_back(new Node(Point<int>(curr_x + 1, curr_y), curr, curr->g_score + 1);     // E
      neighbors.push_back(new Node(Point<int>(curr_x, curr_y - 1), curr, curr->g_score + 1);     // S
      neighbors.push_back(new Node(Point<int>(curr_x - 1, curr_y), curr, curr->g_score + 1);     // W
      neighbors.push_back(new Node(Point<int>(curr_x + 1, curr_y + 1), curr, curr->g_score + distance_between_points(curr->n, Point<int>(curr_x + 1, curr_y + 1))); // NE
      neighbors.push_back(new Node(Point<int>(curr_x - 1, curr_y + 1), curr, curr->g_score + distance_between_points(curr->n, Point<int>(curr_x - 1, curr_y + 1))); // NW
      neighbors.push_back(new Node(Point<int>(curr_x + 1, curr_y - 1), curr, curr->g_score + distance_between_points(curr->n, Point<int>(curr_x + 1, curr_y - 1))); // SE
      neighbors.push_back(new Node(Point<int>(curr_x - 1, curr_y - 1), curr, curr->g_score + distance_between_points(curr->n, Point<int>(curr_x - 1, curr_y - 1))); // SW
      for (int i = 0; i < neighbors.size(); ++i)
      {
        if (closedSet.find(neighbors[i]->n) == closedSet.end())
        { // didn't find in closed set
          if (openSet.find(neighbors[i]) == openSet.end())
          {
            openSet.insert(neighbors[i]);
          }
          else if (neighbors[i]->g_score >= openSet.find(neighbors[i])->g_score)
          {
            continue;
          }
          else
          {
            openSet.remove(neighbors[i]);
            openSet.insert(neighbors[i]);
          }
        }
      }
    }
  }

  return path;
}

robot_path_t reconstruct_path(Node *end, int64_t time_path)
{
  robot_path_t retval;
  Node *traverse = end;
  while (traverse != nullptr)
  {
    pose_xyt_t pose_traverse;
    pose_traverse->x = traverse->n.x;
    pose_traverse->y = traverse->n.y;
    retval.path.push_back(pose_traverse);
    traverse = traverse->p;
  }
  retval.utime = time_path;
  retval.path_length = retval.path.size();
  std::reverse(retval.path.begin(), retval.path.end());
  return retval;
}

float h_score(Node *start, Node *goal, const ObstacleDistanceGrid &distances)
{
  float h = distance_between_points(start->n, goal->n);
  h *= distance.metersPerCell();
}
