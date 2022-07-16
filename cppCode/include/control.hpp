#ifndef CONTROL_HPP
#define CONTROL_HPP

class Control {
    public:
        const float _v0 = 2.0f;
        const float _vMax = 10.0f;
        
        Control(int clientId, int leftHandler, int rightHandler);
        void updateVelocities(float dist, float angle, float &vLeft, float &vRight, float dt);
        void sendVelocities(float vLeft, float vRight);

    private:
        const float _kp = 0.5f;
        const float _kd = 0.2f;
        const float _ki = 0.1f;
        const float _iLimit = 1.0f;

        const float _maxVel = 1.0f;
        const float _deadZone = 0.4f;
        const float _velCoefA = 1.2f;
        const float _velCoefB = -0.1f;
        const float _maxVelDiff = 0.01f;

        int _clientId; 
        int _leftHandler; 
        int _rightHandler;

        float _lastError;
        float _iError;

        float _lastVLeft;
        float _lastVRight;
};

#endif // CONTROL_HPP