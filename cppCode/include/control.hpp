#ifndef CONTROL_HPP
#define CONTROL_HPP

class Control {
    public:
        const float _v0 = 2.0f;
        const float _vMax = 10.0f;
        
        Control();
        void updateVelocities(float dist, float angle, float &vLeft, float &vRight, float dt);

    private:
        const float _kp = 0.5f;
        const float _kd = 0.2f;
        const float _ki = 0.1f;
        const float _iLimit = 1.0f;

        float _lastError;
        float _iError;
};

#endif // CONTROL_HPP