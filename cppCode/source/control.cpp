#include "../include/control.hpp"
#include <iostream>
#include <limits>
#include <cmath>        // std::abs

Control::Control(float kp, float ki, float kd, bool linear){
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _linear = linear;
    _iError = 0.0f;
    _lastError = 0.0f;
}

void Control::updateVelocities(float error, float &vLeft, float &vRight, float dt){
    dt /= 1000;

    _iError += error * dt;

    if(_iError < -_iLimit){
        _iError = -_iLimit;
    } else if (_iError > _iLimit) {
        _iError = _iLimit;
    }

    float pTerm = _kp * error;
    float iTerm = (_ki * _iError);
    float dTerm = _kd * (error - _lastError) / dt;
    _lastError = error;

    float controlTerms = pTerm + iTerm + dTerm;
    // std::cout << controlTerms << " = " << pTerm << " + " << iTerm << " + " << dTerm << std::endl;

    vLeft += controlTerms;
    vRight += _linear ? controlTerms : -controlTerms;
}