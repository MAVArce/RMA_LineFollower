#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

class Actuator {
    public:        
        Actuator(int clientId, int leftHandler, int rightHandler);
        void sendVelocities(float vLeft, float vRight);
        void stop();

    private:
        const float _maxVel = 1.0f;
        const float _deadZone = 0.4f;
        const float _velCoefA = 1.2f;
        const float _velCoefB = -0.1f;
        const float _maxVelDiff = 0.05f;

        int _clientId; 
        int _leftHandler; 
        int _rightHandler;

        float _lastError;
        float _iError;

        float _lastVLeft;
        float _lastVRight;
};

#endif // ACTUATOR_HPP