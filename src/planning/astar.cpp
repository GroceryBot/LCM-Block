#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>


robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    robot_path_t path;
    path.utime = start.utime;
    //First node in path
    path.path.push_back(start);
    path.path_length = path.path.size();
    int width = distances.widthInCells();
    int hight = distances.heightInCells();

    Point<int> start_point = distances.poseToCell(start.x, start.y);
    Node start_node(start_point.x, start_point.y, 0,0,0);
    Point<int> end_point = distances.poseToCell(goal.x, goal.y);
    Node dest_node(end_point.x, end_point.y ,0,0,0);

    std::priority_queue<Node, std::vector<Node>, Compare> pq;
    start_node.visited = true;
    pq.push(start_node);
    Node cur = start_node;
    while(!pq.empty() && !isDestinationReached(cur, dest_node)){
      cur = pq.top();
      pq.pop();
      cur.visited = true;
      //For all the not visted neighbours do:

    }


    return path;
}


static bool isValid(const Node &n, const ObstacleDistanceGrid& distances, const SearchParams& params){
  if (n.visited)
    return false;
  else{
    if(distances(n.X, n.Y)>params.minDistanceToObstacle)
      return true;
    else
      return false;
  }
}


static bool isDestinationReached(const Node &n, const Node &dest){
  return n.X == dest.X && n.Y == dest.Y;
}


static float calculateHscore(const Node &n, const Node &dest,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params){
  float h = sqrt((n.X-dest.X)*(n.X-dest.X) + (n.Y-dest.Y)*(n.Y-dest.Y));
  h *= distances.metersPerCell();
  float cellDistance = distances(n.X, n.Y);
  if(cellDistance > params.minDistanceToObstacle
     && cellDistance < params.maxDistanceWithCost)
       h +=  pow(params.maxDistanceWithCost, params.distanceCostExponent);
  return h;
}
