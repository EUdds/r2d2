#pragma once

#include "TMC2209.hpp"

#include <cstdint>
#include <chrono>
#include <functional>
#include <string>

// Simple dome controller that maps dome angle commands to TMC2209 step pulses.
class DomeMotor {
public:
    struct TimingConfig {
        std::chrono::microseconds pulse_width;
        std::chrono::microseconds min_step_delay;
        // Number of steps to ramp down from a slower start to the target delay.
        std::uint32_t accel_steps;
        // Multiplier applied to the first delay in the ramp (>= 1.0).
        double accel_scale;

        TimingConfig(std::chrono::microseconds pw = std::chrono::microseconds{5},
                     std::chrono::microseconds delay = std::chrono::microseconds{500},
                     std::uint32_t accel_steps_in = 40,
                     double accel_scale_in = 2.0)
            : pulse_width(pw),
              min_step_delay(delay),
              accel_steps(accel_steps_in),
              accel_scale(accel_scale_in) {}
    };

    struct RampConfig {
        double max_speed_dps;        // Maximum speed in deg/s
        double min_speed_dps;        // Minimum speed (starting/ending speed)
        double accel_dps2;           // Acceleration in deg/s^2
        double decel_dps2;           // Deceleration in deg/s^2

        RampConfig(double max_speed = 45.0,
                   double min_speed = 5.0,
                   double accel = 30.0,
                   double decel = 30.0)
            : max_speed_dps(max_speed),
              min_speed_dps(min_speed),
              accel_dps2(accel),
              decel_dps2(decel) {}
    };

    struct PIDConfig {
        double kp;
        double ki;
        double kd;
        double max_speed_dps;   // deg/s cap for safety
        double min_speed_dps;   // avoid stalling at very low speeds
        double integral_limit;  // anti-windup on integral term
        double tolerance_deg;   // acceptable error band
        std::chrono::milliseconds timeout;

        PIDConfig(double kp_in = 0.0,
                  double ki_in = 0.0,
                  double kd_in = 0.0,
                  double max_speed = 45.0,
                  double min_speed = 1.0,
                  double integral_lim = 90.0,
                  double tol_deg = 1.0,
                  std::chrono::milliseconds timeout_in = std::chrono::milliseconds{10000})
            : kp(kp_in),
              ki(ki_in),
              kd(kd_in),
              max_speed_dps(max_speed),
              min_speed_dps(min_speed),
              integral_limit(integral_lim),
              tolerance_deg(tol_deg),
              timeout(timeout_in) {}
    };

    DomeMotor(const std::string& chip_name,
              TMC2209::Pins pins,
              double motor_full_steps_per_rev,
              double gear_ratio,
              bool ms1_high,
              bool ms2_high,
              TimingConfig timing = TimingConfig());

    void enable();
    void disable();

    // Open-loop helpers.
    void rotateDegrees(double degrees);
    void rotateToAngleOpenLoop(double target_deg);

    // Move to angle with trapezoidal speed ramping (smooth accel/decel).
    void rotateToAngleWithRamp(double target_deg, const RampConfig& ramp);

    // Closed-loop move using a user-provided angle reader (e.g., IMU/encoder).
    // Returns true on success within tolerance before timeout.
    bool moveToAnglePID(double target_deg,
                        const PIDConfig& pid,
                        const std::function<double()>& read_angle_deg);

    // Spin the dome at a constant speed (deg/sec) for the given duration (blocking).
    // Positive speed is clockwise, negative is counter-clockwise.
    void seekAtSpeed(double speed_deg_per_sec, std::chrono::milliseconds duration);
    // Streaming velocity helper for control loops (e.g., joystick polling).
    // Call once per control interval with the elapsed time to issue steps for that slice.
    void seekAtSpeedFor(double speed_deg_per_sec, std::chrono::duration<double> dt);

    double currentAngleDeg() const { return current_angle_deg_; }
    void setCurrentAngleDeg(double deg) { current_angle_deg_ = normalizeAngle(deg); }
    double stepsPerDomeDeg() const { return steps_per_dome_deg_; }
    double shortestError(double target, double current) const;

private:
    double microstepsFromPins(bool ms1_high, bool ms2_high) const;
    double normalizeAngle(double deg) const;

    void stepWithAcceleration(bool clockwise,
                              std::uint32_t steps,
                              std::chrono::microseconds base_delay);

    TMC2209 driver_;
    double steps_per_dome_deg_;
    double current_angle_deg_;
    TimingConfig timing_;
};
