#include "occupancy_grid.hpp"
#include "lidar.hpp"
#include <cmath>

OccupancyGrid::OccupancyGrid(int width, int height) {
    grid = cv::Mat(height, width, CV_8UC1, cv::Scalar(127));
}

void OccupancyGrid::markFree(int x, int y) {
    if (inBounds(x, y) && grid.at<uchar>(y, x) != 0)   // un mur connu ne redevient pas libre
        grid.at<uchar>(y, x) = 255;
}

void OccupancyGrid::markOccupied(int x, int y) {
    if (inBounds(x, y)) grid.at<uchar>(y, x) = 0;
}

void OccupancyGrid::updateFromScan(float rx, float ry, float headingDeg,
                                   const std::vector<float>& scan) {
    const int n = static_cast<int>(scan.size());
    if (n == 0) return;

    const cv::Point origin(static_cast<int>(std::lround(rx)), static_cast<int>(std::lround(ry)));
    std::vector<cv::Point> hit(n), freeEnd(n);
    std::vector<bool> isHit(n);

    for (int i = 0; i < n; ++i) {
        float ang = (headingDeg + i) * static_cast<float>(M_PI) / 180.0f;
        float dist = scan[i];
        float c = std::cos(ang), s = std::sin(ang);
        // rayon sans obstacle (portée max) : pas de faux mur
        isHit[i] = dist < LIDAR_MAX_DIST;
        float dFree = std::max(0.0f, dist - 2.0f);   // s'arrête avant le mur
        freeEnd[i] = cv::Point(static_cast<int>(std::lround(rx + c * dFree)),
                               static_cast<int>(std::lround(ry + s * dFree)));
        hit[i]     = cv::Point(static_cast<int>(std::lround(rx + c * dist)),
                               static_cast<int>(std::lround(ry + s * dist)));
    }

    // Zone libre : secteurs triangulaires entre rayons voisins (sans trous
    // entre les rayons quand la distance augmente).
    cv::Mat freeMask = cv::Mat::zeros(grid.size(), CV_8UC1);
    for (int i = 0; i < n; ++i) {
        cv::Point tri[3] = { origin, freeEnd[i], freeEnd[(i + 1) % n] };
        cv::fillConvexPoly(freeMask, tri, 3, cv::Scalar(255));
    }
    cv::Mat known = (grid != 0);                  // ne pas écraser les murs déjà connus
    grid.setTo(255, freeMask & known);

    // Murs : point d'impact + segment entre impacts voisins proches (mur continu)
    for (int i = 0; i < n; ++i) {
        if (!isHit[i]) continue;
        markOccupied(hit[i].x, hit[i].y);
        int j = (i + 1) % n;
        if (isHit[j] && std::abs(scan[i] - scan[j]) < 3.0f)
            cv::line(grid, hit[i], hit[j], cv::Scalar(0), 1);
    }
}

cv::Mat OccupancyGrid::getGrid() const { return grid; }

bool OccupancyGrid::inBounds(int x, int y) const {
    return x >= 0 && x < grid.cols && y >= 0 && y < grid.rows;
}
bool OccupancyGrid::isOccupied(int x, int y) const {
    return inBounds(x, y) && grid.at<uchar>(y, x) == 0;
}
bool OccupancyGrid::isFree(int x, int y) const {
    return inBounds(x, y) && grid.at<uchar>(y, x) == 255;
}
bool OccupancyGrid::isUnknown(int x, int y) const {
    return inBounds(x, y) && grid.at<uchar>(y, x) == 127;
}
