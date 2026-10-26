#ifndef KEYBOARD_HPP_INCLUDED
#define KEYBOARD_HPP_INCLUDED

#include "robot.hpp"

class Keyboard {
public:
    Keyboard();

    /* Traite une touche pressée par l'utilisateur (mode manuel). */
    void processKey(int key, Robot &robot, float& resetX, float& resetY);
};

#endif // KEYBOARD_HPP_INCLUDED
