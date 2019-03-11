#include "queue"
#include "unordered_set"
#include "stack"


#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/point.hpp>
#include <planning/Node.hpp>





robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    std::cout<<"search start (global): "<<start.x<<" "<<start.y<<" goal: "<<goal.x<<" "<<goal.y<<std::endl;
    robot_path_t path;
    path.utime = start.utime;
    //First node in path
    int width = distances.widthInCells();
    int height = distances.heightInCells();
    float metersPerCell = distances.metersPerCell();

    //Initialize search priority queue and visted list (using unordered map)
    std::priority_queue<Node *, std::vector<Node *>, Compare> pq;
    std::unordered_set<Node, hash> open_set;

    //Initialize start point in cell coordinates
    Point<int> end_point = distances.poseToCell(goal.x, goal.y);
    Node dest_node(end_point, nullptr,0);
    Point<int> start_point = distances.poseToCell(start.x, start.y);
    Node start_node(start_point, nullptr,0);
    start_node.f_score =  calculateHscore(start_node, dest_node, distances, params);
    open_set.insert(start_node);


    std::cout<<"search start (Cell): "<<start_node.n.x<<" "<<start_node.n.y<<" goal: "<<dest_node.n.x<<" "<<dest_node.n.y<<std::endl;

    pq.push(&start_node);
    //visited_list.insert(start_node.n);
    Node* cur = &start_node;
    //For debug
    int count = 0;
    while(!pq.empty() && !isDestinationReached(*cur, dest_node) && count<20){
      std::cout<<"pq size: "<<pq.size()<<" open size: "<<open_set.size()<<std::endl;
      cur = pq.top();

      //if(cur->p)
      std::cout<<"current: "<<cur->n.x<<" "<<cur->n.y<<" "<<cur->f_score<<std::endl;//" parent: "<<cur->p->n.x<<" "<<cur->p->n.y<<std::endl;

      //Using 4-neighbour connection here -- can be changed to 8-neighbour
      std::vector<Point<int>> neighbours;
      neighbours.resize(4);
      neighbours[0] = Point<int>(std::max(0,cur->n.x-1), cur->n.y);
      neighbours[3] = Point<int>(cur->n.x, std::max(0,cur->n.y-1));
      neighbours[2] = Point<int>(std::min(cur->n.x+1, width), cur->n.y);
      neighbours[1] = Point<int>(cur->n.x, std::min(cur->n.y+1, height));

      for(unsigned int i=0; i<neighbours.size(); i++){
        if (distances(neighbours[i].x, neighbours[i].y)>params.minDistanceToObstacle){
          std::cout<<"Neighbour #"<<i<<": "<<neighbours[i].x<<" "<<neighbours[i].y<<std::endl;
          Node n(neighbours[i], cur, cur->g_score+metersPerCell);
          auto iter = open_set.find(n);
          if(iter==open_set.end()){
            n.f_score = n.g_score + calculateHscore(n, dest_node, distances, params);
            open_set.insert(n);
            pq.push(&n);
            std::cout<<"Top: "<<pq.top()->n.x<<" "<<pq.top()->n.y<<" "<<pq.top()->f_score<<std::endl;
            std::cout<<"Scores (f g h): "<<n.f_score<<" "<<n.g_score<<" "<<calculateHscore(n, dest_node, distances, params)<<std::endl;
          }
          else{
            if(n.g_score < iter->g_score){
              iter->p = cur;
              iter->f_score = iter->f_score - iter->g_score + n.g_score;
              iter->g_score = n.g_score;
              std::cout<<"Updated Scores (f g h): "<<n.f_score<<" "<<n.g_score<<" "<<calculateHscore(n, dest_node, distances, params)<<std::endl;
            }
         }
        }
      }
      count++;
      pq.pop();
    }

    std::cout<<"Out of the while loop"<<std::endl;

    if(isDestinationReached(*cur, dest_node)){
      while(cur!=nullptr){
        pose_xyt_t cur_pose;
        cur_pose.x = cur->n.x;
        cur_pose.y = cur->n.y;
        //cur_pose.theta = 0;
        path.path.push_back(cur_pose);
        cur = cur->p;
      }
      path.path_length = path.path.size();
    }
    else{
      std::cout<<"No valid path.\n";
    }
    return path;
}


bool isDestinationReached(const Node &n, const Node &dest){
  return n.n == dest.n;
}


float calculateHscore(const Node &n, const Node &dest,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params){
  float h = distance_between_points(n.n, dest.n);
  h *= distances.metersPerCell();
  float cellDistance = distances(n.n.x, n.n.y);
  if(cellDistance > params.minDistanceToObstacle
     && cellDistance < params.maxDistanceWithCost)
       h +=  pow(params.maxDistanceWithCost, params.distanceCostExponent);
  return h;
}
