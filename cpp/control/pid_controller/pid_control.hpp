#pragma once

class PidController {
public:
    PidController(double kp, double ki, double kd);

    void setGains(double kp, double ki, double kd);
    void reset();

    // Compute PID output given error and time step
    // Returns: control output
    double compute(double error, double dt);

    // Getters
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }

private:
    double kp_;
    double ki_;
    double kd_;

    double integral_;
    double prev_error_;
    bool first_run_;
};
