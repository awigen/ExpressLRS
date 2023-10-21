#include "pid.h"
#include "native.h"
unsigned long last_update;

PID::PID(float max, float min, float Kp, float Ki, float Kd)
    : _max(max),
      _min(min),
      _Kp(Kp),
      _Ki(Ki),
      _Kd(Kd),
      error(0),
      _integral(0),
      setpoint(0),
      pv(0),
      output(0)
{
}

void PID::configure(float Kp, float Ki, float Kd, float max, float min)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _max = max;
    _min = min;
}

void PID::reset()
{
    error = 0;
    _integral = 0;
    setpoint = 0;
    pv = 0;
    output = 0;
}

float PID::calculate(float _setpoint, float _pv)
{
    unsigned long now = micros();
    t_delta = now - last_update;
    t_delta = t_delta == 0 ? 1 : t_delta; // Stop any chance of div/0
    float _dt = 1.0 / t_delta;
    last_update = now;

    // Store input for debugging
    setpoint = _setpoint;
    pv = _pv;

    // Calculate error
    float current_error = setpoint - pv;

    // Proportional term
    Pout = _Kp * current_error;

    // Integral term
    _integral += current_error * _dt;

    // Limit the I accumulation within min/max
    _integral = (_integral * _Ki) > _max
                    ? _max / _Ki
                : (_integral * _Ki) < _min
                    ? _min / _Ki
                    : _integral;

    Iout = _Ki * _integral;

    // Derivative term
    float derivative = (current_error - error) / _dt;
    Dout = _Kd * derivative;

    // Calculate total output
    output = Pout + Iout + Dout;

    // Limit output
    if (output > _max)
        output = _max;
    if (output < _min)
        output = _min;

    // Save error to previous error
    error = current_error;

    return output;
}
