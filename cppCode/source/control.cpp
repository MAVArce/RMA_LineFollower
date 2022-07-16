#include "../include/control.hpp"
#include <iostream>
#include <limits>
#include <cmath>        // std::abs

Control::Control(float kp, float ki, float kd){
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _iError = 0.0f;
    _lastError = 0.0f;
}

void Control::updateVelocities(float error, float &vLeft, float &vRight, float dt){
    // dt /= 1000;

    _iError += error * dt;

    if(_iError < -_iLimit){
        _iError = -_iLimit;
    } else if (_iError > _iLimit) {
        _iError = _iLimit;
    }

    // std::cout << "Error: " << angle << std::endl;

    float pTerm = _kp * error;
    float dTerm = _kd * (error - _lastError) / dt;
    float iTerm = (_ki * _iError);

    float controlTerms = pTerm + dTerm + iTerm;
    // std::cout << pTerm << " - " << dTerm << " - " << iTerm << std::endl;

    vLeft += controlTerms;
    vRight -= controlTerms;
}