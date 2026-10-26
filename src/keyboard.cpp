#include "keyboard.hpp"

Keyboard::Keyboard() = default;

// Déplacement manuel (repère carte) : W haut, S bas, A gauche, D droite, R reset.
void Keyboard::processKey(int key, Robot &robot, float& resetX, float& resetY) {
    const float STEP = 3.0f;
    switch (key) {
        case 'w': case 'W': robot.avancer(STEP); break;
        case 's': case 'S': robot.reculer(STEP); break;
        case 'a': case 'A': robot.gauche(STEP);  break;
        case 'd': case 'D': robot.droite(STEP);  break;
        case 'r': case 'R': robot.setX(resetX); robot.setY(resetY); break;
        default: break;
    }
}
