#!/bin/bash
make laptop-only
./bin/botgui &
./bin/slam &
lcm-logplayer-gui data/obstacle_slam_10mx10m_5cm.log

