package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kp, ki, kd;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double setpoint, double measured, double currentTime) {
        double error = setpoint - measured;
        double dt = currentTime - lastTime;

        // Proportional
        double p = kp * error;

        // Integral
        integral += error * dt;
        double i = ki * integral;

        // Derivative
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        double d = kd * derivative;

        // Update state
        lastError = error;
        lastTime = currentTime;

        return p + i + d;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastTime = 0;
    }

    // Allow real-time tuning via FTC Dashboard
    public void setGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
}