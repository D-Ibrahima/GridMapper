#include "camera.hpp"

Camera::Camera(int index) { cap_.open(index, cv::CAP_ANY); }

bool Camera::isOpened() const { return cap_.isOpened(); }

cv::VideoCapture& Camera::getCapture() { return cap_; }
