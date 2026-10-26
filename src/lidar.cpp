#include "lidar.hpp"
#include <cmath>
#include <iostream>

LiDAR::LiDAR(const std::string &filename) {
    fullMap = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (fullMap.empty())
        std::cerr << "[LiDAR] Impossible de charger la carte : " << filename << std::endl;
}

cv::Mat LiDAR::getMap() const { return fullMap; }

float LiDAR::deg2rad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

std::vector<float> LiDAR::scan(float x, float y, float headAngleDeg) const {
    std::vector<float> distances(LIDAR_RAYS, LIDAR_MAX_DIST);
    if (fullMap.empty()) return distances;

    const int rows = fullMap.rows, cols = fullMap.cols;

    for (int r = 0; r < LIDAR_RAYS; ++r) {
        float worldAng = deg2rad(static_cast<float>(r) + headAngleDeg);
        float dx = std::cos(worldAng), dy = std::sin(worldAng);

        for (float d = 1.0f; d <= LIDAR_MAX_DIST; d += 1.0f) {
            int px = static_cast<int>(std::lround(x + dx * d));
            int py = static_cast<int>(std::lround(y + dy * d));

            if (px < 0 || px >= cols || py < 0 || py >= rows ||
                fullMap.at<uchar>(py, px) < 50) {   // hors carte ou mur
                distances[r] = d;
                break;
            }
        }
    }
    return distances;
}
