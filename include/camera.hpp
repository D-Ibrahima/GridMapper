#pragma once
#include <opencv2/opencv.hpp>

class Camera {
public:
    explicit Camera(int index = 0);
    ~Camera() = default;
    Camera(const Camera&) = delete;             // VideoCapture n'est pas copiable
    Camera& operator=(const Camera&) = delete;

    bool isOpened() const;
    cv::VideoCapture& getCapture();             // référence : pas de copie

private:
    cv::VideoCapture cap_;
};
