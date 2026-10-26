#ifndef ROBOT_HPP_INCLUDED
#define ROBOT_HPP_INCLUDED

class Robot {
    private:
        float x_, y_, theta_, headAngleDeg_, vx_, vtheta_;
        
    public:
        Robot(float x, float y);

        void fixerPosition(float nx, float ny, float ntheta);
        void deplacer(float dt);
        void avancer(const float x);
        void reculer(const float x);
        void droite(const float y);
        void gauche(const float y);

        // GETTERS
        float getX() const;
        float getY() const;
        float getTheta() const;
        float getHeadAngleDeg() const;
        float getVx() const;
        float getVtheta() const;

        // SETTERS (Mutateurs)
        void setX(float newX);
        void setY(float newY);
        void setTheta(float newTheta);
        void setHeadAngleDeg(float newHeadAngleDeg);
        void setVx(float newVx);
        void setVtheta(float newVtheta);
};


#endif 