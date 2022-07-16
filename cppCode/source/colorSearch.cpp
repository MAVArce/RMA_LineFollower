#include "../include/colorSearch.hpp"
#include <iostream>
#include <limits>
#include <cmath>     
#include <opencv2/opencv.hpp>
#include <numbers> // std::numbers


extern "C"
{
#include "../remoteApi/extApi.h"
}

ColorSearch::ColorSearch(int clientId, int cameraHandler, int robotHandler){
    _clientId = clientId;
    _cameraHandler = cameraHandler;
    _robotHandler = robotHandler;
}

void ColorSearch::Calibrate(Actuator *actuator){
    _RotateToAngle(M_PI, actuator);
}

float ColorSearch::_getRobotDirection(){
    simxFloat *eulerAngles = (simxFloat*)calloc(3, sizeof(simxFloat));
    simxGetObjectOrientation(_clientId, _robotHandler, -1, eulerAngles, simx_opmode_streaming);

    // std::cout<< "robot direction: " << eulerAngles[0] << ", " << eulerAngles[1] << ", " << eulerAngles[2] << std::endl;
    return (float)eulerAngles[2];
}

float ColorSearch::_angleDiff(float a, float b){
    float diff = a - b;
    if(diff < -(M_PI + 0.01))
        diff += M_PI;

    return diff;
}

void ColorSearch::_RotateToAngle(float angle, Actuator *actuator){
    float error = M_PI;
    float currAngle = _getRobotDirection();
    simxUChar *imageData = nullptr;
    simxInt *cameraRes = (simxInt*)calloc(2,sizeof(simxInt));
    std::cout <<"_RotateToAngle: " << angle << std::endl;

    while(error > 0.05){        
        currAngle = _getRobotDirection();
        error = std::abs(_angleDiff(currAngle, angle));

        std::cout<< "robot direction: " << currAngle << ", error: " << error << std::endl;
        float vel = (error / M_PI)*0.6 + 0.4;
        actuator->sendVelocities(-vel, vel);

        simxGetVisionSensorImage(_clientId, _cameraHandler, cameraRes, &imageData, 0, simx_opmode_streaming);
        if(cameraRes[0] > 0){
            cv::Mat myMat(cameraRes[1],cameraRes[0],CV_8UC3,&imageData[0]);
            cv::flip(myMat, myMat, 0);
            cv::cvtColor(myMat, myMat, cv::COLOR_RGB2BGR);
            cv::imshow("CameraFrontal", myMat);
        }
        cv::waitKey(5);
    }

    for(int i = 0; i < 5; i++){
        actuator->sendVelocities(0, 0);

        simxGetVisionSensorImage(_clientId, _cameraHandler, cameraRes, &imageData, 0, simx_opmode_streaming);
        if(cameraRes[0] > 0){
            cv::Mat myMat(cameraRes[1],cameraRes[0],CV_8UC3,&imageData[0]);
            cv::flip(myMat, myMat, 0);
            cv::cvtColor(myMat, myMat, cv::COLOR_RGB2BGR);
            cv::imshow("CameraFrontal", myMat);
        }
        cv::waitKey(5);
    }
}