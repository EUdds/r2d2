#include "DomeMotor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <thread>

DomeMotor::DomeMotor(const std::string& chip_name,
                     TMC2209::Pins pins,
                     double motor_full_steps_per_rev,
                     double gear_ratio,
                     bool ms1_high,
                     bool ms2_high,
                     TimingConfig timing)
    : driver_(chip_name, pins),
      steps_per_dome_deg_(0.0),
      current_angle_deg_(0.0),
      timing_(timing) {
    if (!pins.has_ms1 || !pins.has_ms2) {
        throw std::invalid_argument("MS1 and MS2 pins are required to derive microsteps");
    }
    const double microsteps = microstepsFromPins(ms1_high, ms2_high);
    driver_.setMicrostepPins(ms1_high, ms2_high);
    steps_per_dome_deg_ = (motor_full_steps_per_rev * microsteps * gear_ratio) / 360.0;
    if (steps_per_dome_deg_ <= 0.0) {
        throw std::invalid_argument("steps_per_dome_deg must be positive");
    }
}

void DomeMotor::enable() { driver_.enable(); }

void DomeMotor::disable() { driver_.disable(); }

void DomeMotor::rotateDegrees(double degrees) {
    if (degrees == 0.0) {
        return;
    }
    const bool clockwise = degrees > 0.0;
    const auto steps = static_cast<std::uint32_t>(std::llround(std::abs(degrees) * steps_per_dome_deg_));
    if (steps == 0) {
        return;
    }
    stepWithAcceleration(clockwise, steps, timing_.min_step_delay);

    const double delta_angle = steps / steps_per_dome_deg_;
    current_angle_deg_ = normalizeAngle(current_angle_deg_ + (clockwise ? delta_angle : -delta_angle));
}

void DomeMotor::rotateToAngleOpenLoop(double target_deg) {
    const double error = shortestError(target_deg, current_angle_deg_);
    rotateDegrees(error);
}

void DomeMotor::rotateToAngleWithRamp(double target_deg, const RampConfig& ramp) {
    const double total_distance = shortestError(target_deg, current_angle_deg_);
    if (std::abs(total_distance) < 0.01) {
        return;  // Already at target
    }

    const bool clockwise = total_distance > 0.0;
    const double abs_distance = std::abs(total_distance);

    // Calculate trapezoidal profile parameters
    const double v_min = std::abs(ramp.min_speed_dps);
    const double v_max = std::abs(ramp.max_speed_dps);
    const double accel = std::abs(ramp.accel_dps2);
    const double decel = std::abs(ramp.decel_dps2);

    // Distance needed to accelerate from v_min to v_max
    const double accel_distance = (v_max * v_max - v_min * v_min) / (2.0 * accel);
    // Distance needed to decelerate from v_max to v_min
    const double decel_distance = (v_max * v_max - v_min * v_min) / (2.0 * decel);

    // Check if we can reach max speed or if it's a triangular profile
    const double min_distance_for_trapezoid = accel_distance + decel_distance;
    bool is_triangular = abs_distance < min_distance_for_trapezoid;

    double peak_speed = v_max;
    double actual_accel_distance = accel_distance;
    double actual_decel_distance = decel_distance;
    double cruise_distance = 0.0;

    if (is_triangular) {
        // Calculate peak speed for triangular profile
        peak_speed = std::sqrt(v_min * v_min + abs_distance * accel * decel / (accel + decel));
        actual_accel_distance = (peak_speed * peak_speed - v_min * v_min) / (2.0 * accel);
        actual_decel_distance = (peak_speed * peak_speed - v_min * v_min) / (2.0 * decel);
    } else {
        // Trapezoidal profile - cruise at max speed
        cruise_distance = abs_distance - accel_distance - decel_distance;
    }

    // Execute motion profile step by step
    double distance_traveled = 0.0;
    driver_.setDirection(clockwise);

    while (distance_traveled < abs_distance) {
        // Determine current speed based on position in profile
        double current_speed;
        if (distance_traveled < actual_accel_distance) {
            // Acceleration phase
            current_speed = std::sqrt(v_min * v_min + 2.0 * accel * distance_traveled);
            current_speed = std::min(current_speed, peak_speed);
        } else if (distance_traveled < actual_accel_distance + cruise_distance) {
            // Cruise phase
            current_speed = peak_speed;
        } else {
            // Deceleration phase
            const double remaining_distance = abs_distance - distance_traveled;
            current_speed = std::sqrt(v_min * v_min + 2.0 * decel * remaining_distance);
            current_speed = std::max(current_speed, v_min);
        }

        // Calculate delay between steps based on current speed
        const double steps_per_second = current_speed * steps_per_dome_deg_;
        const double min_delay_us = static_cast<double>(timing_.min_step_delay.count());
        const double requested_delay_us = steps_per_second > 0.0 ? (1'000'000.0 / steps_per_second) : min_delay_us;
        const auto step_delay = std::chrono::microseconds(
            static_cast<long long>(std::max(min_delay_us, requested_delay_us)));

        // Take one step
        driver_.stepOnce(timing_.pulse_width);

        // Update position
        const double step_distance = 1.0 / steps_per_dome_deg_;
        distance_traveled += step_distance;
        current_angle_deg_ = normalizeAngle(current_angle_deg_ + (clockwise ? step_distance : -step_distance));

        // Wait before next step (unless we're at the end)
        if (distance_traveled < abs_distance) {
            std::this_thread::sleep_for(step_delay);
        }
    }
}

bool DomeMotor::moveToAnglePID(double target_deg,
                               const PIDConfig& pid,
                               const std::function<double()>& read_angle_deg) {
    auto last_time = std::chrono::steady_clock::now();
    auto start_time = last_time;
    double integral = 0.0;
    double last_error = 0.0;
    bool first = true;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::max(1e-3, std::chrono::duration<double>(now - last_time).count());  // seconds
        last_time = now;

        current_angle_deg_ = normalizeAngle(read_angle_deg());
        const double error = shortestError(target_deg, current_angle_deg_);
        if (std::abs(error) <= pid.tolerance_deg) {
            return true;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time) > pid.timeout) {
            return false;
        }

        integral += error * dt;
        const double integral_limit = std::abs(pid.integral_limit);
        integral = std::clamp(integral, -integral_limit, integral_limit);

        const double derivative = first ? 0.0 : (error - last_error) / dt;
        const double output_dps = pid.kp * error + pid.ki * integral + pid.kd * derivative;  // deg/s
        last_error = error;
        first = false;

        double speed = output_dps;
        const double abs_speed = std::clamp(std::abs(speed), pid.min_speed_dps, pid.max_speed_dps);
        const bool clockwise = speed >= 0.0;

        // Steps to issue this cycle, sized from requested speed and elapsed time.
        const double steps_this_cycle = abs_speed * dt * steps_per_dome_deg_;
        std::uint32_t step_count = static_cast<std::uint32_t>(std::ceil(steps_this_cycle));
        if (step_count == 0) {
            step_count = 1;
        }

        const double steps_per_second = abs_speed * steps_per_dome_deg_;
        const double min_delay_us = static_cast<double>(timing_.min_step_delay.count());
        const double requested_delay_us =
            steps_per_second > 0.0 ? (1'000'000.0 / steps_per_second) : min_delay_us;
        const auto step_delay = std::chrono::microseconds(
            static_cast<long long>(std::max(min_delay_us, requested_delay_us)));

        stepWithAcceleration(clockwise, step_count, step_delay);

        const double delta_angle = step_count / steps_per_dome_deg_;
        current_angle_deg_ = normalizeAngle(current_angle_deg_ + (clockwise ? delta_angle : -delta_angle));
    }
}

void DomeMotor::seekAtSpeed(double speed_deg_per_sec, std::chrono::milliseconds duration) {
    seekAtSpeedFor(speed_deg_per_sec, std::chrono::duration<double>(duration));
}

void DomeMotor::seekAtSpeedFor(double speed_deg_per_sec, std::chrono::duration<double> dt) {
    const double seconds = dt.count();
    if (speed_deg_per_sec == 0.0 || seconds <= 0.0) {
        return;
    }

    const bool clockwise = speed_deg_per_sec > 0.0;
    const double abs_speed = std::abs(speed_deg_per_sec);
    const double steps_per_second = abs_speed * steps_per_dome_deg_;
    if (steps_per_second <= 0.0) {
        return;
    }

    // Derive step timing from requested speed and enforce minimum delay.
    const double min_delay_us = static_cast<double>(timing_.min_step_delay.count());
    const double requested_delay_us = 1'000'000.0 / steps_per_second;
    const auto step_delay = std::chrono::microseconds(
        static_cast<long long>(std::max(min_delay_us, requested_delay_us)));

    const double steps_this_cycle = steps_per_second * seconds;
    std::uint32_t step_count = static_cast<std::uint32_t>(std::ceil(steps_this_cycle));
    if (step_count == 0) {
        step_count = 1;
    }

    stepWithAcceleration(clockwise, step_count, step_delay);

    const double delta_angle = step_count / steps_per_dome_deg_;
    current_angle_deg_ = normalizeAngle(current_angle_deg_ + (clockwise ? delta_angle : -delta_angle));
}

void DomeMotor::stepWithAcceleration(bool clockwise,
                                     std::uint32_t steps,
                                     std::chrono::microseconds base_delay) {
    if (steps == 0) {
        return;
    }

    const double base_delay_us = std::max(1.0, static_cast<double>(base_delay.count()));
    const double start_delay_us = std::max(base_delay_us, base_delay_us * std::max(1.0, timing_.accel_scale));
    const std::uint32_t ramp_steps = std::min<std::uint32_t>(steps, timing_.accel_steps);

    driver_.setDirection(clockwise);
    for (std::uint32_t i = 0; i < steps; ++i) {
        driver_.stepOnce(timing_.pulse_width);
        if (i + 1 >= steps) {
            break;
        }

        double delay_us = base_delay_us;
        if (ramp_steps > 0 && i < ramp_steps) {
            const double t = static_cast<double>(i) / ramp_steps;
            delay_us = start_delay_us - (start_delay_us - base_delay_us) * t;
            if (delay_us < base_delay_us) {
                delay_us = base_delay_us;
            }
        }
        const auto delay = std::chrono::microseconds(
            static_cast<long long>(std::llround(delay_us)));
        std::this_thread::sleep_for(delay);
    }
}

double DomeMotor::microstepsFromPins(bool ms1_high, bool ms2_high) const {
    // Truth table:
    // MS1:0 MS2:0 -> 1/8 rev per step -> 8 microsteps
    // MS1:1 MS2:1 -> 1/16 rev per step -> 16 microsteps
    // MS1:1 MS2:0 -> 1/32 rev per step -> 32 microsteps
    // MS1:0 MS2:1 -> 1/64 rev per step -> 64 microsteps
    if (!ms1_high && !ms2_high) {
        return 8.0;
    }
    if (ms1_high && ms2_high) {
        return 16.0;
    }
    if (ms1_high && !ms2_high) {
        return 32.0;
    }
    return 64.0;  // !ms1_high && ms2_high
}

double DomeMotor::normalizeAngle(double deg) const {
    double result = std::fmod(deg, 360.0);
    if (result < 0.0) {
        result += 360.0;
    }
    return result;
}

double DomeMotor::shortestError(double target, double current) const {
    const double norm_target = normalizeAngle(target);
    const double norm_current = normalizeAngle(current);
    double error = norm_target - norm_current;
    if (error > 180.0) {
        error -= 360.0;
    } else if (error < -180.0) {
        error += 360.0;
    }
    return error;
}
