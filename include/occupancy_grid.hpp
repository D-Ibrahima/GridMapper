#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

class OccupancyGrid {
private:
    cv::Mat grid;  // 0 = mur, 255 = libre, 127 = inconnu

public:
    OccupancyGrid(int width, int height);

    void markFree(int x, int y);
    void markOccupied(int x, int y);
    // scan[i] = distance du rayon i, dans la direction (headingDeg + i) degrés
    void updateFromScan(float rx, float ry, float headingDeg, const std::vector<float>& scan);

    cv::Mat getGrid() const;

    bool inBounds(int x, int y) const;
    bool isOccupied(int x, int y) const;
    bool isFree(int x, int y) const;
    bool isUnknown(int x, int y) const;
};
