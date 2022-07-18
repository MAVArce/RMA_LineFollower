#ifndef CONTROL_HPP
#define CONTROL_HPP

class Control {
    public:
        Control(float kp, float ki, float kd, bool linear);
        void updateVelocities(float error, float &vLeft, float &vRight, float dt);

    private:
        const float _iLimit = 0.1f;
        bool _linear;
        float _kp;
        float _ki;
        float _kd;
        float _lastError;
        float _iError;
};

#endif // CONTROL_HPP