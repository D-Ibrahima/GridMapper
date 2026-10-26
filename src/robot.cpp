#include "robot.hpp"
#include <cmath>

float Robot::getX() const { return x_; }
float Robot::getY() const { return y_; }
float Robot::getTheta() const { return theta_; }
float Robot::getHeadAngleDeg() const { return headAngleDeg_; }
float Robot::getVx() const { return vx_; }
float Robot::getVtheta() const { return vtheta_; }

void Robot::setX(float newX) { x_ = newX; }
void Robot::setY(float newY) { y_ = newY; }
void Robot::setTheta(float newTheta) { theta_ = newTheta; }
void Robot::setHeadAngleDeg(float newHeadAngleDeg) { headAngleDeg_ = newHeadAngleDeg; }
void Robot::setVx(float newVx) { vx_ = newVx; }
void Robot::setVtheta(float newVtheta) { vtheta_ = newVtheta; }

Robot::Robot(float x, float y)
    : x_(x), y_(y), theta_(180.0f), headAngleDeg_(180.0f), vx_(0), vtheta_(0) {}

void Robot::fixerPosition(float nx, float ny, float ntheta) {
    x_ = nx;
    y_ = ny;
    headAngleDeg_ = std::fmod(std::fmod(ntheta, 360.0f) + 360.0f, 360.0f);
    theta_ = headAngleDeg_;
}

// vx_ en px/s (le long du cap), vtheta_ en degrés/s (positif = sens horaire à l'écran)
void Robot::deplacer(float dt) {
    float rad = headAngleDeg_ * static_cast<float>(M_PI) / 180.0f;
    x_ += vx_ * std::cos(rad) * dt;
    y_ += vx_ * std::sin(rad) * dt;

    headAngleDeg_ = std::fmod(headAngleDeg_ + vtheta_ * dt, 360.0f);
    if (headAngleDeg_ < 0.0f) headAngleDeg_ += 360.0f;
    theta_ = headAngleDeg_;
}

// Déplacements manuels dans le repère de la carte (y vers le bas en OpenCV)
void Robot::avancer(const float d)  { y_ -= d; }
void Robot::reculer(const float d)  { y_ += d; }
void Robot::droite(const float d)   { x_ += d; }
void Robot::gauche(const float d)   { x_ -= d; }
