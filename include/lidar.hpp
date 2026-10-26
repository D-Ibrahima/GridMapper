#ifndef LIDAR_HPP_INCLUDED
#define LIDAR_HPP_INCLUDED

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

inline constexpr int   LIDAR_RAYS     = 360;    // 1 rayon par degré
inline constexpr float LIDAR_MAX_DIST = 300.0f; // portée en pixels

// Convention : repère image (y vers le bas). L'angle croît dans le sens horaire :
// rayon 0 = devant, 90 = droite, 180 = derrière, 270 = gauche.
class LiDAR {
public:
    explicit LiDAR(const std::string &filename);
    cv::Mat getMap() const;
    std::vector<float> scan(float x, float y, float headAngleDeg) const;

private:
    cv::Mat fullMap;
    static float deg2rad(float deg);
};
#endif
