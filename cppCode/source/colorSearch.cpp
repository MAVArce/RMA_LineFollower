#include "../include/colorSearch.hpp"
#include <iostream>
#include <limits>
#include <cmath>     
#include <opencv2/opencv.hpp>


extern "C"
{
#include "../remoteApi/extApi.h"
}

ColorSearch::ColorSearch(int clientId, int robotHandler, Vision *visionCtrl){
    _clientId = clientId;
    _robotHandler = robotHandler;
    _visionCtrl = visionCtrl;
    oriLandmark = 0;
}

void ColorSearch::_calibrateGreen(){
    _greenFilter.hueMin =  45; _greenFilter.hueMax = 135;
    _greenFilter.satMin = 120; _greenFilter.satMax = 255;
    _greenFilter.valMin =  50; _greenFilter.valMax = 255;
    _greenFilter.isSplit = false;

    int hmax_high = _calculateValue(&_greenFilter, &(_greenFilter.hueMax),  1, 180, 0);
    int hmax_low  = _calculateValue(&_greenFilter, &(_greenFilter.hueMax), -1, 180, 0);

    int hmin_high = _calculateValue(&_greenFilter, &(_greenFilter.hueMin),  1, 180, 0);
    int hmin_low  = _calculateValue(&_greenFilter, &(_greenFilter.hueMin), -1, 180, 0);

    int smax_high = _calculateValue(&_greenFilter, &(_greenFilter.satMax),  1, 255, 0);
    int smax_low  = _calculateValue(&_greenFilter, &(_greenFilter.satMax), -1, 255, 0);

    int smin_high = _calculateValue(&_greenFilter, &(_greenFilter.satMin),  1, 255, 0);
    int smin_low  = _calculateValue(&_greenFilter, &(_greenFilter.satMin), -1, 255, 0);
    
    // std::cout << "hmax_high: " << hmax_high << ", hmax_low: " << hmax_low << ", hmin_high: " << hmin_high << 
    //             ", hmin_low: " << hmin_low << ", smax_high: " << smax_high << ", smax_low: " << smax_low << 
    //             ", smin_high: " << smin_high << ", smin_low: " << smin_low << std::endl;
    
    _greenFilter.hueMin = ((hmin_high * 3) + (hmin_low * 1)) / 4;
    _greenFilter.hueMax = ((hmax_high * 1) + (hmax_low * 3)) / 4;
    _greenFilter.satMin = ((smin_high * 3) + (smin_low * 1)) / 4; 
    _greenFilter.satMax = ((smax_high * 1) + (smax_low * 3)) / 4;
}

void ColorSearch::_calibrateRed(){
    _redFilterBot.hueMin =  0; _redFilterBot.hueMax = 30;
    _redFilterBot.satMin = 120; _redFilterBot.satMax = 255;
    _redFilterBot.valMin =  50; _redFilterBot.valMax = 255;
    _redFilterBot.isSplit = true;

    _redFilterTop.hueMin =  150; _redFilterTop.hueMax = 179;
    _redFilterTop.satMin = 120; _redFilterTop.satMax = 255;
    _redFilterTop.valMin =  50; _redFilterTop.valMax = 255;
    _redFilterTop.isSplit = true;

    int hmaxBot_high = _calculateValue(&_redFilterBot, &(_redFilterBot.hueMax),  1, 180, 0);
    int hmaxBot_low  = _calculateValue(&_redFilterBot, &(_redFilterBot.hueMax), -1, 180, 0);

    int smaxBot_high = _calculateValue(&_redFilterBot, &(_redFilterBot.satMax),  1, 255, 0);
    int smaxBot_low  = _calculateValue(&_redFilterBot, &(_redFilterBot.satMax), -1, 255, 0);

    int sminBot_high = _calculateValue(&_redFilterBot, &(_redFilterBot.satMin),  1, 255, 0);
    int sminBot_low  = _calculateValue(&_redFilterBot, &(_redFilterBot.satMin), -1, 255, 0);
    
    _redFilterBot.hueMax = ((hmaxBot_high * 1) + (hmaxBot_low * 2)) / 3;
    _redFilterBot.satMin = ((sminBot_high * 2) + (sminBot_low * 1)) / 3; 
    _redFilterBot.satMax = ((smaxBot_high * 1) + (smaxBot_low * 2)) / 3;
    
    // std::cout << "hmaxBot_high: " << hmaxBot_high << ", hmaxBot_low: " << hmaxBot_low << ", smaxBot_high: " << smaxBot_high << 
    //             ", smaxBot_low: " << smaxBot_low << ", sminBot_high: " << sminBot_high << ", sminBot_low: " << sminBot_low << std::endl;

    int hminTop_high = _calculateValue(&_redFilterTop, &(_redFilterTop.hueMin),  1, 180, 0);
    int hminTop_low  = _calculateValue(&_redFilterTop, &(_redFilterTop.hueMin), -1, 180, 0);

    int smaxTop_high = _calculateValue(&_redFilterTop, &(_redFilterTop.satMax),  1, 255, 0);
    int smaxTop_low  = _calculateValue(&_redFilterTop, &(_redFilterTop.satMax), -1, 255, 0);

    int sminTop_high = _calculateValue(&_redFilterTop, &(_redFilterTop.satMin),  1, 255, 0);
    int sminTop_low  = _calculateValue(&_redFilterTop, &(_redFilterTop.satMin), -1, 255, 0);

    // std::cout << "hminTop_high: " << hminTop_high << ", hminTop_low: " << hminTop_low << ", smaxTop_high: " << smaxTop_high << 
    //             ", smaxTop_low: " << smaxTop_low << ", sminTop_high: " << sminTop_high << ", sminTop_low: " << sminTop_low << std::endl;
    
    _redFilterTop.hueMin = ((hminTop_high * 2) + (hminTop_low * 1)) / 3;
    _redFilterTop.satMin = ((sminTop_high * 2) + (sminTop_low * 1)) / 3; 
    _redFilterTop.satMax = ((smaxTop_high * 1) + (smaxTop_low * 2)) / 3;

}

void ColorSearch::Calibrate(Actuator *actuator, cv::Point *pt, int *dist){
    float originalAngle = GetRobotDirection();
    // RotateToAngle(_angleSum(originalAngle,  M_PI), actuator);
    
    cv::Mat imageOriginal = _visionCtrl->getImageFrontal();
    cv::Mat imageNew = _visionCtrl->getImageFrontal();
    cv::imwrite("calibrationImg.png", imageOriginal);
    _calibrateGreen();
    _calibrateRed();

    oriLandmark = GetRobotDirection();

    FindLandmark(pt, dist);

    RotateToAngle(_angleSum(originalAngle,  M_PI), actuator);
    // RotateToAngle(originalAngle, actuator);
}

void ColorSearch::SearchLandmarkInEnvinronment(Actuator *actuator, cv::Point *pt, int *dist){
    float angleStep = M_PI / 5;
    float nextAngle = _angleSum(GetRobotDirection(), angleStep);
    float angleToRotate = (2 * M_PI) - angleStep;

    FindLandmark(pt, dist);

    while(angleToRotate > 0 && pt->x == -1 && pt->y == -1){
        RotateToAngle(nextAngle, actuator);
        nextAngle += angleStep;
        angleToRotate -= angleStep;

        FindLandmark(pt, dist);
    }
}

void ColorSearch::FindLandmark(cv::Point *pt, int *dist, cv::Mat *image){
    *pt = cv::Point(-1,-1);
    *dist = -1;

    const int qtdContours = 10; 
    std::vector<std::vector<cv::Point>> greenLandmark, redLandmark;
    std::vector<std::vector<cv::Point>> greenContours = detect(_greenFilter, qtdContours);
    std::vector<std::vector<cv::Point>> redContours = detect(_redFilterBot, _redFilterTop, 2*qtdContours);
    
    if(greenContours.empty() || redContours.size() < 2)
        return;

    if(image != nullptr){
        cv::drawContours(*image, redContours, -1, cv::Scalar(0,0,0), 2, cv::LINE_8);
        cv::drawContours(*image, greenContours, -1, cv::Scalar(255,0,0), 2, cv::LINE_8);
    }

    cv::Rect greenRect;
    cv::Moments redM1, redM2;
    cv::Point redCenter1, redCenter2, middlePoint;
    
    bool foundLandmark = false;
    for(int i = 0; i < (int)redContours.size() && !foundLandmark; i++){        
        redM1 = cv::moments(redContours[i],true);
        redCenter1 = cv::Point(redM1.m10/redM1.m00, redM1.m01/redM1.m00);
        // double areaRed1 = cv::contourArea()
        
        for(int j = 0; j < (int)redContours.size() && !foundLandmark; j++){
            if(i == j)
                continue;

            redM2 = cv::moments(redContours[j],true);
            redCenter2 = cv::Point(redM2.m10/redM2.m00, redM2.m01/redM2.m00);
            middlePoint = cv::Point((redCenter1.x + redCenter2.x)/2, (redCenter1.y + redCenter2.y)/2);

            for(int k = 0; k < (int)greenContours.size() && !foundLandmark; k++){
                greenRect = cv::boundingRect(greenContours[k]);

                if(middlePoint.x < greenRect.x || middlePoint.x > (greenRect.x + greenRect.width) ||
                    middlePoint.y < greenRect.y || middlePoint.y > (greenRect.y + greenRect.height))
                    continue;
                    
                foundLandmark = true;
            }
        }
    }
    // std::cout<< "middlePoint: " << middlePoint << ", greenRect: " << greenRect << std::endl;
    if(foundLandmark != true)
        return;

    *pt = middlePoint;
    *dist = sqrt((redCenter1.x - redCenter2.x) * (redCenter1.x - redCenter2.x) + (redCenter1.y - redCenter2.y) * (redCenter1.y - redCenter2.y));

    if(image != nullptr){
        cv::circle(*image, redCenter1, 5, cv::Scalar( 0, 0, 0 ), cv::FILLED, cv::LINE_8 );
        cv::circle(*image, redCenter2, 5, cv::Scalar( 0, 0, 0 ), cv::FILLED, cv::LINE_8 );
        cv::circle(*image, middlePoint, 5, cv::Scalar( 255, 255, 0 ), cv::FILLED, cv::LINE_8 );
        cv::rectangle(*image, greenRect, cv::Scalar( 0, 255, 255), cv::FILLED, cv::LINE_8);
    }
}

std::vector<std::vector<cv::Point>> ColorSearch::detect(Filter filter){
    if(filter.isSplit)
        return detect(filter, 2);
    
    return detect(filter, 1);
}

std::vector<std::vector<cv::Point>> ColorSearch::detect(Filter filter1, Filter filter2, int qtd){
    cv::Mat res, res1, res2;
    cv::Mat image = _visionCtrl->getImageFrontal();
    cvtColor(image, image, cv::COLOR_BGR2HSV);        

    cv::inRange(image, cv::Scalar(filter1.hueMin, filter1.satMin, filter1.valMin), 
                cv::Scalar(filter1.hueMax, filter1.satMax, filter1.valMax), res1);

    cv::inRange(image, cv::Scalar(filter2.hueMin, filter2.satMin, filter2.valMin), 
                cv::Scalar(filter2.hueMax, filter2.satMax, filter2.valMax), res2);

    res = res1 + res2;
    std::vector<std::vector<cv::Point>> contours = _visionCtrl->getBiggestContours(res, qtd);
    return contours;    
}

std::vector<std::vector<cv::Point>> ColorSearch::detect(Filter filter, int qtd){
    cv::Mat filterRes;
    cv::Mat image = _visionCtrl->getImageFrontal();
    cvtColor(image, image, cv::COLOR_BGR2HSV);        

    

    cv::inRange(image, cv::Scalar(filter.hueMin, filter.satMin, filter.valMin), 
                cv::Scalar(filter.hueMax, filter.satMax, filter.valMax), filterRes);

    std::vector<std::vector<cv::Point>> contours = _visionCtrl->getBiggestContours(filterRes, qtd);
    return contours;    
}

int ColorSearch::_calculateValue(Filter *filter, int *valueToChange, int step, int max, int min){
    std::vector<std::vector<cv::Point>> contours = detect(*filter);
    if(contours.empty())
        return *valueToChange;

    int qtdAreas = 1;
    if(filter->isSplit)
        qtdAreas = 2;

    std::vector<double> originalAreas(qtdAreas);
    std::vector<cv::Point> originalCenters(qtdAreas);
    
    for(int i = 0; i < qtdAreas; i++){
        cv::Moments m = cv::moments(contours[i],true);
        originalAreas[i] =  cv::contourArea(contours[i], false);
        originalCenters[i] = cv::Point(m.m10/m.m00, m.m01/m.m00);
    }

    bool finished = false;
    cv::Mat image = _visionCtrl->getImageFrontal();
    int xDiffMax = image.rows/100;
    int yDiffMax = image.cols/100;
    int originalValue = *valueToChange;
    while(!finished){
        *valueToChange += step;
        if(*valueToChange < min || *valueToChange > max)
            break;

        contours = detect(*filter);
        if(contours.empty())
            break;
        
        for(int i = 0; i < qtdAreas; i++){
            cv::Moments m = cv::moments(contours[i],true);
            double newArea = cv::contourArea(contours[i], false);
            cv::Point newCenter(m.m10/m.m00, m.m01/m.m00);


            double areaRatio = newArea / originalAreas[i];
            if(areaRatio > 1.01 || areaRatio < 0.99)
                finished = true;

            int xDiff = newCenter.x - originalCenters[i].x;
            int yDiff = newCenter.y - originalCenters[i].y;
            
            if(xDiff > xDiffMax || xDiff < -xDiffMax || yDiff > yDiffMax || yDiff < -yDiffMax)
                finished = true;
        }
    }

    int result = *valueToChange - step;
    *valueToChange = originalValue;

    return result;
}

float ColorSearch::GetRobotDirection(){
    simxFloat *eulerAngles = (simxFloat*)calloc(3, sizeof(simxFloat));
    simxGetObjectOrientation(_clientId, _robotHandler, -1, eulerAngles, simx_opmode_streaming);
    while(eulerAngles[0] == 0){
        // std::cout << "GetRobotDirection loop" << std::endl;
        simxGetObjectOrientation(_clientId, _robotHandler, -1, eulerAngles, simx_opmode_streaming);
        extApi_sleepMs(5);
    }
    return (float)eulerAngles[2];
}

float ColorSearch::AngleDiff(float a, float b){
    float diff = a - b;    
    if(diff < -M_PI){
        diff += M_PI * 2;
    } else if(diff > M_PI){
        diff -= M_PI * 2;
    }

    return diff;
}

float ColorSearch::_angleSum(float a, float b){
    float sum = a + b;
    if(sum < -M_PI)
        sum = (sum + M_PI) * -1;
    if(sum > M_PI)
        sum = (sum - M_PI) * -1;

    return sum;
}

void ColorSearch::RotateToAngle(float angle, Actuator *actuator){
    float error = M_PI;
    float currAngle = GetRobotDirection();

    while(abs(error) > 0.05){        
        currAngle = GetRobotDirection();
        error = AngleDiff(currAngle, angle);

        float vel = (error)*0.6 + (error > 0 ? 0.4 : -0.4);

        actuator->sendVelocities(vel, -vel);

        cv::Mat image = _visionCtrl->getImageFrontal();
        cv::imshow("CameraFrontal", image);
        cv::waitKey(5);
    }

    actuator->stop();
}

