#include "pid_control.hpp"

PidController::PidController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0), first_run_(true)
{
}

void PidController::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_run_ = true;
}

double PidController::compute(double error, double dt)
{
    // Proportional term
    double p_term = kp_ * error;

    // Integral term
    integral_ += error * dt;
    double i_term = ki_ * integral_;

    // Derivative term
    double d_term = 0.0;
    if (!first_run_ && dt > 0.0)
    {
        double derivative = (error - prev_error_) / dt;
        d_term = kd_ * derivative;
    }

    prev_error_ = error;
    first_run_ = false;

    return p_term + i_term + d_term;
}
