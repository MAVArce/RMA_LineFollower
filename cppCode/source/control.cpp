#include "../include/control.hpp"
#include <iostream>
#include <limits>

Control::Control(){
    _iError = 0.0f;
    _lastError = 0.0f;
}

void Control::updateVelocities(float dist, float angle, float &vLeft, float &vRight, float dt){
    float error = angle;

    _iError += error * dt;

    if(_iError < -_iLimit){
        _iError = -_iLimit;
    } else if (_iError > _iLimit) {
        _iError = _iLimit;
    }

    float pTerm = _kp * error;
    float dTerm = _kd * (error - _lastError) / dt;
    float iTerm = (_ki * _iError);

    float controlTerms = pTerm + dTerm + iTerm;

    vLeft = _v0 - controlTerms;
    vRight = _v0 + controlTerms;

    vLeft = std::min(vLeft, _vMax);
    vRight = std::min(vRight, _vMax);
}