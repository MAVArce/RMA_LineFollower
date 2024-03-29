#ifndef COLOR_HPP
#define COLOR_HPP

#include "../include/actuator.hpp"
#include "../include/vision.hpp"

class ColorSearch {
    public:
        ColorSearch(int clientId, int robotHandler, Vision *visionCtrl);

        void Calibrate(Actuator *actuator, cv::Point *pt, int *dist);
        void FindLandmark(cv::Point *pt, int *dist, cv::Mat *image = nullptr);
        void SearchLandmarkInEnvinronment(Actuator *actuator, cv::Point *pt, int *dist);
        float GetRobotDirection();
        float AngleDiff(float a, float b);
        void RotateToAngle(float angle, Actuator *actuator);

        float oriLandmark;

    private:
        struct Filter{
            bool isSplit;
            int hueMin; int hueMax;
            int satMin; int satMax;
            int valMin; int valMax;
        } _redFilterTop, _redFilterBot, _greenFilter;

        int _clientId;
        int _cameraHandler;
        int _robotHandler;

        Vision *_visionCtrl;

        float _angleSum(float a, float b);

        int _calculateValue(Filter *filter, int *valueToChange, int step, int max, int min);        
        void _calibrateGreen();
        void _calibrateRed();
        // std::vector<std::vector<cv::Point>> detectSingle(Filter filter);
        // std::vector<std::vector<cv::Point>> detectSplit(Filter filter);
        // std::vector<std::vector<cv::Point>> detect(Filter filter);
        std::vector<std::vector<cv::Point>> detect(Filter filter);
        std::vector<std::vector<cv::Point>> detect(Filter filter, int qtd);
        std::vector<std::vector<cv::Point>> detect(Filter filter1, Filter filter2, int qtd);
};

#endif // COLOR_HPP