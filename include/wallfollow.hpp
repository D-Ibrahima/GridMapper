#ifndef WALLFOLLOW_HPP
#define WALLFOLLOW_HPP

#include "robot.hpp"
#include "lidar.hpp"
#include <cmath>

class WallFollow {
public:
    WallFollow(LiDAR* l);
    
    void suivre(Robot &robot);

private:
    LiDAR* lidar;
};

#endif