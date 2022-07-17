#include "../include/actuator.hpp"
#include <iostream>
#include <limits>
#include <cmath>        // std::abs

extern "C"
{
#include "../remoteApi/extApi.h"
}

Actuator::Actuator(int clientId, int leftHandler, int rightHandler){
    _lastVLeft = 0.0f;
    _lastVRight = 0.0f;

    _clientId = clientId;
    _leftHandler = leftHandler;
    _rightHandler = rightHandler;
}

void Actuator::sendVelocities(float vLeft, float vRight){

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
    // std::cout << "actual VLeft: " << realVLeft << std::endl;
    // std::cout << "actual VRight: " << realVRight << std::endl;
    // std::cout << std::endl;

    simxSetJointTargetVelocity(_clientId, _leftHandler, (simxFloat)realVLeft, simx_opmode_streaming);
    simxSetJointTargetVelocity(_clientId, _rightHandler, (simxFloat)realVRight, simx_opmode_streaming);
}

void Actuator::stop(){
    sendVelocities(0,0);
    extApi_sleepMs(1);        

    for(int i = 0; i < 3; i++){
        int curr_sim_time = (int)simxGetLastCmdTime(_clientId);
        int dt = 0 ;
        int last_sim_time = curr_sim_time;
        while(dt == 0 || _lastVLeft != 0 || _lastVRight != 0){
            curr_sim_time = (int)simxGetLastCmdTime(_clientId);
            dt = curr_sim_time - last_sim_time;
            last_sim_time = curr_sim_time;

            sendVelocities(0,0);
            extApi_sleepMs(1);        
        }
    }
}