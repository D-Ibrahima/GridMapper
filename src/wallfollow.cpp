#include <cmath>
#include "wallfollow.hpp"
#include <algorithm>
#include <iostream>

WallFollow::WallFollow(LiDAR* l) : lidar(l) {}

// Suivi de mur À GAUCHE. Convention : angle horaire -> d[270] = gauche, d[90] = droite,
// w > 0 = tourner à droite, w < 0 = tourner à gauche (deg/s).
void WallFollow::suivre(Robot &robot)
{
    auto d = lidar->scan(robot.getX(), robot.getY(), robot.getHeadAngleDeg());

    if (d.size() < 360) {
        robot.setVx(0);
        robot.setVtheta(0);
        return;
    }

    const float front = d[0];
    const float left  = d[270];
    const float right = d[90];

    float min_front = front;
    for (int a = -30; a <= 30; a++)
        min_front = std::min(min_front, d[(a + 360) % 360]);

    const float DISTANCE_MUR_IDEALE = 60.0f;
    const float SEUIL_OBSTACLE = 50.0f;
    const float VITESSE_NORMALE = 30.0f;
    const float VITESSE_VIRAGE  = 18.0f;
    const float KP = 0.8f;            // deg/s par pixel d'erreur de distance
    const float KH = 1.5f;            // deg/s par degré d'erreur d'angle
    const float W_MAX = 50.0f;

    float v = 0.0f, w = 0.0f;

    if (front < 30.0f && left < 30.0f && right < 30.0f) {
        // coincé : reculer en tournant
        v = -20.0f;
        w = 60.0f;
    }
    else if (min_front < SEUIL_OBSTACLE) {
        // obstacle devant : mur à gauche -> pivoter vers la DROITE
        v = 5.0f;
        w = 70.0f;
    }
    else if (left > DISTANCE_MUR_IDEALE + 40.0f) {
        // mur perdu / virage à gauche : courber vers la GAUCHE pour le retrouver
        v = VITESSE_NORMALE;
        w = -10.0f;
    }
    else {
        // Régulation : distance perpendiculaire ET angle par rapport au mur.
        // b = rayon latéral (270), a = rayon 30° plus en avant (300).
        const float THETA = 30.0f * static_cast<float>(M_PI) / 180.0f;
        const float a = d[300], b = left;
        float alpha = 0.0f;                                   // angle du mur (rad), < 0 : on pointe vers le mur
        if (a < LIDAR_MAX_DIST - 1.0f)
            alpha = std::atan2(a * std::cos(THETA) - b, a * std::sin(THETA));
        float dist = b * std::cos(alpha);
        float erreur = dist - DISTANCE_MUR_IDEALE;            // > 0 : trop loin
        float alphaDeg = alpha * 180.0f / static_cast<float>(M_PI);
        w = -KP * erreur - KH * alphaDeg;
        w = std::max(-W_MAX, std::min(W_MAX, w));
        v = (std::abs(erreur) > 15.0f) ? VITESSE_VIRAGE : VITESSE_NORMALE;
    }

    robot.setVx(v);
    robot.setVtheta(w);
}
