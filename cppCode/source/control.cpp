#include "../include/control.hpp"
#include <iostream>
#include <limits>
#include <cmath>        // std::abs

extern "C"
{
#include "../remoteApi/extApi.h"
}

Control::Control(int clientId, int leftHandler, int rightHandler){
    _iError = 0.0f;
    _lastError = 0.0f;

    _lastVLeft = 0.0f;
    _lastVRight = 0.0f;

    _clientId = clientId;
    _leftHandler = leftHandler;
    _rightHandler = rightHandler;
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

void Control::sendVelocities(float vLeft, float vRight){

    float realVLeft = std::abs(vLeft);
    float realVRight = std::abs(vRight);
    
    
    if(realVLeft < _deadZone)
        realVLeft = 0.0;
    else
        realVLeft = realVLeft*_velCoefA + _velCoefB;
    
    if(realVRight < _deadZone)
        realVRight = 0.0;
    else
        realVRight = realVRight*_velCoefA + _velCoefB;
    
    if(realVLeft > _maxVel)
        realVLeft = _maxVel;

    if(realVRight > _maxVel)
        realVRight = _maxVel;
    
    if(vLeft < 0)
        realVLeft = realVLeft*(-1.0);
    
    if(vRight < 0)
        realVRight = realVRight*(-1.0);
    
    
    float diffLeft = realVLeft - _lastVLeft;
    if(diffLeft > _maxVelDiff)
        realVLeft = _lastVLeft + _maxVelDiff;
    else if(diffLeft < (-_maxVelDiff))
        realVLeft = _lastVLeft - _maxVelDiff;

    
    float diffRight = realVRight - _lastVRight;
    if(diffRight > _maxVelDiff)
        realVRight = _lastVRight + _maxVelDiff;
    else if(diffRight < (-_maxVelDiff))
        realVRight = _lastVRight - _maxVelDiff;
    
    _lastVLeft = realVLeft;
    _lastVRight = realVRight;

    simxSetJointTargetVelocity(_clientId, _leftHandler, (simxFloat)realVLeft, simx_opmode_streaming);
    simxSetJointTargetVelocity(_clientId, _rightHandler, (simxFloat)realVRight, simx_opmode_streaming);
}