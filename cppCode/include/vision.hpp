#ifndef VISION_HPP
#define VISION_HPP

extern "C"
{
#include "../remoteApi/extApi.h"
}

#include <opencv2/opencv.hpp>
class Vision {
    public:        
        Vision(int clientId, int linhaHandler, int frontalHandler);
        cv::Mat getImageLinha();
        cv::Mat getImageFrontal();

    private:
        int _clientId; 
        const int _resizeFactor = 20;

        struct camInfo {
            int handler;
            int channels;
            simxInt *res;

            cv::Mat smallNoise;
            cv::Mat biggerNoise;
            cv::Mat biggerNoiseResized;
        } _linhaInfo, _frontalInfo;


        void _getCamResolution(camInfo *cam);
        void _applyNoise(camInfo cam, cv::Mat *image);
};

#endif // VISION_HPP