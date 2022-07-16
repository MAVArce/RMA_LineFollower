#include "../include/vision.hpp"
#include <iostream>
#include <limits>
#include <cmath>
#include <opencv2/opencv.hpp>


extern "C"
{
#include "../remoteApi/extApi.h"
}

Vision::Vision(int clientId, int linhaHandler, int frontalHandler){
    _clientId = clientId;

    _linhaInfo.handler = linhaHandler; _linhaInfo.channels = CV_8UC3;
    _frontalInfo.handler = frontalHandler; _frontalInfo.channels = CV_8UC3;

    _getCamResolution(&_linhaInfo);
    _getCamResolution(&_frontalInfo);

    _linhaInfo.smallNoise = cv::Mat::zeros (_linhaInfo.res[1], _linhaInfo.res[0], _linhaInfo.channels);
    _linhaInfo.biggerNoise = cv::Mat::zeros (_linhaInfo.res[1] / _resizeFactor, _linhaInfo.res[0] / _resizeFactor, _linhaInfo.channels);
    _linhaInfo.biggerNoiseResized = cv::Mat::zeros (_linhaInfo.res[1], _linhaInfo.res[0], _linhaInfo.channels);

    _frontalInfo.smallNoise = cv::Mat::zeros (_frontalInfo.res[1], _frontalInfo.res[0], _frontalInfo.channels);
    _frontalInfo.biggerNoise = cv::Mat::zeros (_frontalInfo.res[1] / _resizeFactor, _frontalInfo.res[0] / _resizeFactor, _frontalInfo.channels);
    _frontalInfo.biggerNoiseResized = cv::Mat::zeros (_frontalInfo.res[1], _frontalInfo.res[0], _frontalInfo.channels);
}

cv::Mat Vision::getImageLinha(){
    simxUChar *imageData = nullptr;
    simxGetVisionSensorImage(_clientId, _linhaInfo.handler, _linhaInfo.res, &imageData, 0, simx_opmode_streaming);
    if(_linhaInfo.res[0] == 0)
        return cv::Mat::zeros(10, 10, _linhaInfo.channels);

    cv::Mat image(_linhaInfo.res[1],_linhaInfo.res[0],_linhaInfo.channels, &imageData[0]);
    cv::flip(image, image, 0);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    _applyNoise(_linhaInfo, &image);

    return image;
}

cv::Mat Vision::getImageFrontal(){
    simxUChar *imageData = nullptr;
    simxGetVisionSensorImage(_clientId, _frontalInfo.handler, _frontalInfo.res, &imageData, 0, simx_opmode_streaming);
    if(_frontalInfo.res[0] == 0)
        return cv::Mat::zeros(10, 10, _frontalInfo.channels);

    cv::Mat image(_frontalInfo.res[1],_frontalInfo.res[0],_frontalInfo.channels,&imageData[0]);
    cv::flip(image, image, 0);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    _applyNoise(_frontalInfo, &image);

    return image;
}

void Vision::_getCamResolution(camInfo *cam){
    simxUChar *imageData = nullptr;
    cam->res = (simxInt*)calloc(2,sizeof(simxInt));

    while(cam->res[0] == 0){
        simxGetVisionSensorImage(_clientId, cam->handler, cam->res, &imageData, 0, simx_opmode_streaming);
        extApi_sleepMs(1);
    }
}

void Vision::_applyNoise(camInfo cam, cv::Mat *image){
    cv::randu(cam.smallNoise, 0, 255);
    cv::randu(cam.biggerNoise, 0, 255);
    cv::resize(cam.biggerNoise, cam.biggerNoiseResized, cv::Size(), _resizeFactor, _resizeFactor, cv::INTER_NEAREST);

    *image = *image + cam.smallNoise*0.2 + cam.biggerNoiseResized*0.1;
}