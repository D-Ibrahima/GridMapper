#include "exploration.hpp"
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <queue>
#include <opencv2/imgproc.hpp>

Exploration::Exploration(Robot* r, LiDAR* l, OccupancyGrid* m)
    : robot(r), lidar(l), map(m) {}



bool Exploration::planToFrontier(const Robot& rob, float& headingDeg, float& pathLen) const
{
    constexpr int   CELL = 5;          // taille d'une cellule de planification (px)
    constexpr float INFLATE = 9.0f;    // marge de sécurité autour des murs (px)
    constexpr int   LOOKAHEAD = 6;     // point de passage visé : 6 cellules plus loin

    const cv::Mat g = map->getGrid();
    const int W = g.cols / CELL, H = g.rows / CELL;

    cv::Mat occ = (g == 0), inflated;
    int k = static_cast<int>(INFLATE) * 2 + 1;
    cv::dilate(occ, inflated, cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k}));

    auto traversable = [&](int cx, int cy) {
        int x = cx * CELL + CELL / 2, y = cy * CELL + CELL / 2;
        return g.at<uchar>(y, x) == 255 && inflated.at<uchar>(y, x) == 0;
    };
    auto unknownAt = [&](int cx, int cy) {
        if (cx < 0 || cy < 0 || cx >= W || cy >= H) return false;
        return g.at<uchar>(cy * CELL + CELL / 2, cx * CELL + CELL / 2) == 127;
    };

    const int sx = std::min(W - 1, std::max(0, static_cast<int>(rob.getX()) / CELL));
    const int sy = std::min(H - 1, std::max(0, static_cast<int>(rob.getY()) / CELL));

    std::vector<int> parent(W * H, -2);   // -2 = non visité
    std::queue<int> q;
    parent[sy * W + sx] = -1;
    q.push(sy * W + sx);

    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    int found = -1;
    while (!q.empty() && found < 0) {
        int cur = q.front(); q.pop();
        int cx = cur % W, cy = cur / W;
        if (cur != sy * W + sx) {
            for (int i = 0; i < 4; ++i)
                if (unknownAt(cx + dx[i], cy + dy[i])) { found = cur; break; }
            if (found >= 0) break;
        }
        for (int i = 0; i < 4; ++i) {
            int nx = cx + dx[i], ny = cy + dy[i];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            int ni = ny * W + nx;
            if (parent[ni] != -2 || !traversable(nx, ny)) continue;
            parent[ni] = cur;
            q.push(ni);
        }
    }
    if (found < 0) return false;

    // remonter le chemin : liste depuis la frontière jusqu'au robot
    std::vector<int> path;
    for (int c = found; c != -1; c = parent[c]) path.push_back(c);
    std::reverse(path.begin(), path.end());      // path[0] = cellule du robot
    pathLen = static_cast<float>(path.size() * CELL);
    int wp = path[std::min<size_t>(LOOKAHEAD, path.size() - 1)];
    float tx = (wp % W) * CELL + CELL / 2.0f, ty = (wp / W) * CELL + CELL / 2.0f;
    headingDeg = std::atan2(ty - rob.getY(), tx - rob.getX()) * 180.0f / static_cast<float>(M_PI);
    return true;
}

void Exploration::explorer(Robot &rob)
{
    // ========= 1. LIDAR =========
    auto d = lidar->scan(rob.getX(), rob.getY(), rob.getHeadAngleDeg());
    if (d.size() < 360) {
        rob.setVx(0);
        rob.setVtheta(0);
        return;
    }

    // ========= 2. CARTE =========
    map->updateFromScan(rob.getX(), rob.getY(), rob.getHeadAngleDeg(), d);

    // ========= 3. CAPTEURS UTILES =========
    float ang = rob.getHeadAngleDeg() * static_cast<float>(M_PI) / 180.f;
    int nx = static_cast<int>(rob.getX() + std::cos(ang) * 8);
    int ny = static_cast<int>(rob.getY() + std::sin(ang) * 8);
    bool out = !map->inBounds(nx, ny);

    float minAhead = 9999.f;
    for (int a = -20; a <= 20; ++a)
        minAhead = std::min(minAhead, d[(a + 360) % 360]);

    const float left  = d[270];   // 270 = gauche, 90 = droite (angle horaire)
    const float right = d[90];
    // sens de rotation vers le côté le plus dégagé : - = gauche, + = droite
    const float openDir = (left > right) ? -1.f : +1.f;

    constexpr float HARD = 20;   // mur imminent
    constexpr float MID  = 30;   // obstacle proche
    constexpr float SOFT = 50;   // obstacle lointain

    // Plan vers la frontière (le chemin évite déjà les murs avec une marge)
    float targetHeading = 0.f, pathLen = 0.f;
    const bool havePlan = planToFrontier(rob, targetHeading, pathLen);
    float planAngle = 0.f;                       // - = gauche
    if (havePlan) {
        planAngle = targetHeading - rob.getHeadAngleDeg();
        while (planAngle > 180.0f)  planAngle -= 360.0f;
        while (planAngle < -180.0f) planAngle += 360.0f;
    }

    // ========= 4. ANTI-BLOCAGE =========
    if (!hasPrev_) { prevX_ = rob.getX(); prevY_ = rob.getY(); hasPrev_ = true; }
    float moved = std::hypot(rob.getX() - prevX_, rob.getY() - prevY_);
    stagn_ = (moved < 0.05f) ? stagn_ + 1 : 0;
    prevX_ = rob.getX();
    prevY_ = rob.getY();

    if (stagn_ > 25) {
        std::cout << "[UNBLOCK] recule + rotation\n";
        // ne pas reculer dans un mur : vérifier derrière (rayon 180)
        rob.setVx(d[180] > 30.f ? -25.f : 25.f);
        rob.setVtheta(+60);
        if (stagn_ > 40) stagn_ = 0;
        return;
    }

    // ========= 5. HORS GRILLE =========
    if (out) {
        rob.setVx(0);
        rob.setVtheta(+60);
        return;
    }

    // ========= 6. OBSTACLE IMMINENT =========
    if (map->isOccupied(nx, ny) || minAhead < HARD) {
        std::cout << "[URGENT] Évitement obstacle côté "
                  << (openDir < 0 ? "gauche" : "droite") << "\n";
        rob.setVx(-5);
        rob.setVtheta(openDir * 60);   // tourner vers le côté dégagé
        return;
    }

    // ========= 7. OBSTACLE PROCHE =========
    if (minAhead < MID) {
        std::cout << "[PROCHE] Ralentissement + contournement\n";
        rob.setVx(8);
        // si un chemin existe, le suivre (il contourne déjà l'obstacle), sinon côté dégagé
        rob.setVtheta(havePlan ? std::max(-60.f, std::min(60.f, planAngle * 2.0f)) : openDir * 45);
        return;
    }

    // ========= 8-9. NAVIGATION VERS LA FRONTIÈRE LA PLUS PROCHE (par le chemin) =========
    if (havePlan) {
        const float angle_target = planAngle;

        std::cout << "[FRONTIER] chemin=" << pathLen << "px, angle=" << angle_target << "\n";

        float omega = angle_target * 2.0f;  // deg/s
        // répulsion latérale : s'écarter d'un mur trop proche sur le côté
        constexpr float SIDE = 30.0f;
        if (left  < SIDE) omega += (SIDE - left)  * 3.0f;   // mur à gauche -> vers la droite (+)
        if (right < SIDE) omega -= (SIDE - right) * 3.0f;   // mur à droite -> vers la gauche (-)
        omega = std::max(-90.0f, std::min(90.0f, omega));
        rob.setVx(std::abs(angle_target) < 25.0f ? 40.0f : 10.0f);
        rob.setVtheta(omega);
        return;
    }

    // ========= 10. SECOURS : AUCUNE FRONTIÈRE ACCESSIBLE (exploration terminée) =========
    if (minAhead < SOFT) {
        // aller vers le côté le plus dégagé
        rob.setVx(25);
        rob.setVtheta(openDir * 30);
    } else {
        rob.setVx(50);
        exploreCnt_++;
        // petite variation aléatoire de cap tous les 60 cycles
        rob.setVtheta(exploreCnt_ % 60 == 0 ? ((std::rand() % 100) - 50) * 0.8f : 0.f);
    }
}
