#ifndef COLOR_HPP
#define COLOR_HPP

#include "../include/actuator.hpp"
#include "../include/vision.hpp"

class ColorSearch {
    public:
        ColorSearch(int clientId, int robotHandler, Vision *visionCtrl);

        void Calibrate(Actuator *actuator);

    private:
        int greenHue;
        int redHue;

        int _clientId;
        int _cameraHandler;
        int _robotHandler;

        Vision *_visionCtrl;

        float _getRobotDirection();
        float _angleDiff(float a, float b);

        void _RotateToAngle(float angle, Actuator *actuator);
};

#endif // COLOR_HPP