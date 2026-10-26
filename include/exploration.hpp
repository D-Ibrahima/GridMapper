#ifndef EXPLORATION_HPP
#define EXPLORATION_HPP

#include "robot.hpp"
#include "lidar.hpp"
#include "occupancy_grid.hpp"

class Exploration {
public:
    Exploration(Robot* r, LiDAR* l, OccupancyGrid* m);

    // Exploration basée sur la détection de frontières (libre -> inconnu).
    // Commande rob.setVx (px/s) et rob.setVtheta (deg/s, + = sens horaire).
    void explorer(Robot &rob);

private:
    // BFS dans la grille jusqu'à la frontière libre/inconnu la plus proche (par le chemin).
    // Retourne le cap absolu (deg) du prochain point de passage et la longueur du chemin.
    bool planToFrontier(const Robot& rob, float& headingDeg, float& pathLen) const;

    Robot* robot;
    LiDAR* lidar;
    OccupancyGrid* map;

    // état anti-blocage (avant : variables static partagées entre les appels)
    float prevX_ = 0.f, prevY_ = 0.f;
    bool  hasPrev_ = false;
    int   stagn_ = 0;
    int   exploreCnt_ = 0;
};

#endif
