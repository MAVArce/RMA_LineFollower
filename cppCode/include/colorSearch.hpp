#ifndef COLOR_HPP
#define COLOR_HPP

#include "../include/actuator.hpp"

class ColorSearch {
    public:
        ColorSearch(int clientId, int cameraHandler, int robotHandler);

        void Calibrate(Actuator *actuator);

    private:
        int greenHue;
        int redHue;

        int _clientId;
        int _cameraHandler;
        int _robotHandler;

        float _getRobotDirection();
        float _angleDiff(float a, float b);

        void _RotateToAngle(float angle, Actuator *actuator);
};

#endif // COLOR_HPP