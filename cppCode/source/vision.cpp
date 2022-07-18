#include "../include/vision.hpp"
#include <iostream>
#include <limits>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>


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

std::vector<cv::Point> Vision::getBiggestContour(cv::Mat image){
    std::vector<std::vector<cv::Point>> lst = getBiggestContours(image, 1);
    // std::cout << "lst size: " << lst.size() <<  ", area: " << cv::contourArea(lst[0], false) <<std::endl;

    if(lst.empty())
        return std::vector<cv::Point>();

    return lst[0];
}

std::vector<std::vector<cv::Point>> Vision::getBiggestContours(cv::Mat image, int qtd){
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, 1, cv::CHAIN_APPROX_NONE);

    std::vector<int> largestIdx;
    std::vector<double> largestArea;
    // largestIdx.reserve(qtd); largestArea.reserve(qtd);

    for(int i = 0; i < qtd; i++){
        largestIdx.push_back(-1);
        largestArea.push_back(0.0);
    }

    for (int i = 0; i< (int)contours.size(); i++) // iterate through each contour. 
    {
        double a = cv::contourArea(contours[i], false);  //  Find the area of contour
        for(int j = 0; j < qtd; j++){
            if (a > largestArea[j]){
                for(int k = j+1; k < qtd; k++){
                    largestIdx[k] = largestIdx[k-1];
                    largestArea[k] = largestArea[k-1];
                }
                largestIdx[j] = i;
                largestArea[j] = a;       
                break;
            }
        }
    }
    std::vector<std::vector<cv::Point>> result;
    for(int i = 0; i < qtd; i++){
        if(largestIdx[i] != -1)
            result.push_back(contours[largestIdx[i]]);
    }
    return result;
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
    // return;
    cv::randu(cam.smallNoise, 0, 255);
    cv::randu(cam.biggerNoise, 0, 255);
    cv::resize(cam.biggerNoise, cam.biggerNoiseResized, cv::Size(), _resizeFactor, _resizeFactor, cv::INTER_NEAREST);

    *image = *image + cam.smallNoise*0.2 + cam.biggerNoiseResized*0.1;
}