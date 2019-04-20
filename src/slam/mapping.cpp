#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <math.h>
#include <unordered_set>

struct pair_hash
{
    inline std::size_t operator()(const std::pair<int, int> &v) const
    {
        return v.first * 31 + v.second;
    }
};


std::vector<std::pair<int, int>> generateOccupied(const OccupancyGrid& map) {
    std::vector<std::pair<int, int>> occupied;
    occupied.clear();
    for (int i = 0; i < map.widthInCells(); ++i) {
        for (int j = 0; j < map.heightInCells(); ++j) {
            if (map.logOdds(i, j) > 70) {
                occupied.push_back({i, j});
            }
        }
    }
    return occupied;
}

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
    : kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds)
{
}
int metersToCellX(float x, OccupancyGrid &map)
{
    return std::floor(x * map.cellsPerMeter()) + (map.widthInCells() / 2);
}
int metersToCellY(float y, OccupancyGrid &map)
{
    return std::floor(y * map.cellsPerMeter()) + (map.heightInCells() / 2);
}
void plotLineLow(float x0, float y0, float x1, float y1, OccupancyGrid &map, float kMissOdds, std::unordered_set< std::pair<int, int>,  pair_hash> &missCells)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float yi = 1;
    if (dy < 0)
    {
        yi = -1;
        dy = -dy;
    }
    float D = 2 * dy - dx;
    float cur_y = y0;
    float cur_x = x0;
    for (; cur_x < x1; cur_x += 0.05)
    {
        if (missCells.find({metersToCellX(cur_x, map), metersToCellY(cur_y, map)}) == missCells.end())
        {
            int val = map.logOdds(metersToCellX(cur_x, map), metersToCellY(cur_y, map));
            if (val <= -127 || val - kMissOdds <= -127)
                val = -127;
            else
                val -= kMissOdds;
            map.setLogOdds(metersToCellX(cur_x, map), metersToCellY(cur_y, map), val);
            missCells.insert({metersToCellX(cur_x, map), metersToCellY(cur_y, map)});
        }
        if (D > 0)
        {
            cur_y += yi * 0.05;
            D -= 2 * dx;
        }
        D += 2 * dy;
    }
}
void plotLineHigh(float x0, float y0, float x1, float y1, OccupancyGrid &map, float kMissOdds, std::unordered_set< std::pair<int, int>,  pair_hash> &missCells)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float xi = 1;
    if (dx < 0)
    {
        xi = -1;
        dx = -dx;
    }
    float D = 2 * dx - dy;
    float cur_y = y0;
    float cur_x = x0;
    for (; cur_y < y1; cur_y += 0.05)
    {
        if (missCells.find({metersToCellX(cur_x, map), metersToCellY(cur_y, map)}) == missCells.end())
        {
            int val = map.logOdds(metersToCellX(cur_x, map), metersToCellY(cur_y, map));
            if (val <= -127 || val - kMissOdds <= -127)
                val = -127;
            else
                val -= kMissOdds;
            map.setLogOdds(metersToCellX(cur_x, map), metersToCellY(cur_y, map), val);
            missCells.insert({metersToCellX(cur_x, map), metersToCellY(cur_y, map)});
        }
        if (D > 0)
        {
            cur_x += xi * 0.05;
            D -= 2 * dy;
        }
        D += 2 * dx;
    }
}

float calculateX(float distance, float theta)
{
    return distance * cos(theta);
}
float calculateY(float distance, float theta)
{
    return distance * sin(theta);
}
float distance2(float x1, float y1, float x2, float y2) {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1));
}
void Mapping::updateMap(const lidar_t &scan, const pose_xyt_t &pose, OccupancyGrid &map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!started)
    {
        map.reset();
        last_pose = pose;
        started = true;
        return;
    }
    std::unordered_set< std::pair<int, int>,  pair_hash> missCells;
    if(std::abs(pose.theta -last_pose.theta) > .45){
      return;
    }
    // std::vector<std::pair<int, int>> occupied = generateOccupied(map);
    MovingLaserScan ml_scan(scan, last_pose, pose);
    for (unsigned int i = 0; i < ml_scan.size(); ++i)
    {
        float x0 = ml_scan[i].origin.x;
        float y0 = ml_scan[i].origin.y;
        float x1 = (ml_scan[i].origin.x + calculateX(ml_scan[i].range, ml_scan[i].theta));
        float y1 = (ml_scan[i].origin.y + calculateY(ml_scan[i].range, ml_scan[i].theta));
        
        if (std::abs(y1 - y0) < std::abs(x1 - x0))
        {
            if (x0 > x1)
            {
                plotLineLow(x1, y1, x0, y0, map, kMissOdds_, missCells);
            }
            else
            {
                plotLineLow(x0, y0, x1, y1, map, kMissOdds_, missCells);
            }
        }
        else
        {
            if (y0 > y1)
            {
                plotLineHigh(x1, y1, x0, y0, map, kMissOdds_, missCells);
            }
            else
            {
                plotLineHigh(x0, y0, x1, y1, map, kMissOdds_, missCells);
            }
        }

        // float min_distance = 1000;
        // std::pair<int, int> min_location;
        // for (unsigned int k = 0; k < occupied.size(); ++k)
        // {
        //     float dist = distance2(metersToCellX(x1, map), metersToCellY(y1, map), occupied[k].first, occupied[k].second);
        //     if (dist < min_distance) {
        //         min_location = occupied[k];
        //         min_distance = dist;
        //     }
        //     if (min_distance == 0) break;
        // }
        // if (min_distance > 1) {
            int val = map.logOdds(metersToCellX(x1, map), metersToCellY(y1, map));
            if (val >= 127 || val + kHitOdds_ >= 127)
                val = 127;
            else
                if(ml_scan[i].range > 5.0)
                    val += kHitOdds_/3;
                else
                    val += kHitOdds_;
            map.setLogOdds(metersToCellX(x1, map), metersToCellY(y1, map), val);
        // }
        // else {
        //     int val = map.logOdds(min_location.first, min_location.second);
        //     if (val >= 127 || val + kHitOdds_ >= 127)
        //         val = 127;
        //     else
        //         val += kHitOdds_;
        //     map.setLogOdds(min_location.first, min_location.second, val);
        // }
    }
    last_pose = pose;
}
