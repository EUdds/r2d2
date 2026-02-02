#include "DomeMotor.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cmath>

#ifndef GPIO_CHIP
#define GPIO_CHIP "gpiochip4"
#endif

#ifndef STEP_PIN
#define STEP_PIN 17
#endif

#ifndef DIR_PIN
#define DIR_PIN 4
#endif

#ifndef ENABLE_PIN
#define ENABLE_PIN 5
#endif

#ifndef ENABLE_ACTIVE_HIGH
#define ENABLE_ACTIVE_HIGH 0
#endif

#ifndef MS1_PRESENT
#define MS1_PRESENT 1
#endif

#ifndef MS1_PIN
#define MS1_PIN 27
#endif

#ifndef MS2_PRESENT
#define MS2_PRESENT 1
#endif

#ifndef MS2_PIN
#define MS2_PIN 22
#endif

#ifndef MS1_HIGH
#define MS1_HIGH 0
#endif

#ifndef MS2_HIGH
#define MS2_HIGH 0
#endif

#ifndef MOTOR_FULL_STEPS
#define MOTOR_FULL_STEPS 207
#endif

#ifndef GEAR_RATIO
#define GEAR_RATIO (124 / 15)
#endif

namespace {

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " (--degrees <deg> | --to <deg> [--ramp]) [--current <deg>]\n"
              << "       " << prog << " --pins\n"
              << "\nOptions:\n"
              << "  --degrees <deg>       Rotate by relative degrees (simple move)\n"
              << "  --to <deg>            Rotate to absolute angle (simple move by default)\n"
              << "  --current <deg>       Set current angle before moving\n"
              << "  --ramp                Use trapezoidal speed ramping (smooth accel/decel)\n"
              << "  --max-speed <dps>     Maximum speed in deg/s (default: 45.0)\n"
              << "  --min-speed <dps>     Starting/ending speed in deg/s (default: 5.0)\n"
              << "  --accel <dps2>        Acceleration in deg/s^2 (default: 30.0)\n"
              << "  --decel <dps2>        Deceleration in deg/s^2 (default: 30.0)\n"
              << "  --pins                Show pin configuration\n"
              << "\nPins and microstep mode are set at compile time via -D macros (use --copt to override).\n";
}

void print_pins() {
    std::cout << "GPIO Chip: " << GPIO_CHIP << "\n";
    std::cout << "Pin Configuration:\n";
    std::cout << "  STEP_PIN:           " << STEP_PIN << "\n";
    std::cout << "  DIR_PIN:            " << DIR_PIN << "\n";
    std::cout << "  ENABLE_PIN:         " << ENABLE_PIN << "\n";
    std::cout << "  ENABLE_ACTIVE_HIGH: " << (ENABLE_ACTIVE_HIGH ? "true" : "false") << "\n";
    std::cout << "  MS1_PRESENT:        " << (MS1_PRESENT ? "true" : "false") << "\n";
    if (MS1_PRESENT) {
        std::cout << "  MS1_PIN:            " << MS1_PIN << "\n";
        std::cout << "  MS1_HIGH:           " << (MS1_HIGH ? "true" : "false") << "\n";
    }
    std::cout << "  MS2_PRESENT:        " << (MS2_PRESENT ? "true" : "false") << "\n";
    if (MS2_PRESENT) {
        std::cout << "  MS2_PIN:            " << MS2_PIN << "\n";
        std::cout << "  MS2_HIGH:           " << (MS2_HIGH ? "true" : "false") << "\n";
    }
    std::cout << "\nMotor Configuration:\n";
    std::cout << "  MOTOR_FULL_STEPS:   " << MOTOR_FULL_STEPS << "\n";
    std::cout << "  GEAR_RATIO:         " << GEAR_RATIO << "\n";

    // Calculate microstep mode
    int microstep_mode = 1;
    if (MS1_PRESENT && MS2_PRESENT) {
        if (!MS1_HIGH && !MS2_HIGH) microstep_mode = 1;
        else if (MS1_HIGH && !MS2_HIGH) microstep_mode = 2;
        else if (!MS1_HIGH && MS2_HIGH) microstep_mode = 4;
        else microstep_mode = 8;
    }
    std::cout << "  Microstep Mode:     1/" << microstep_mode << "\n";
}

bool parse_double(const char* text, double& out) {
    try {
        out = std::stod(text);
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    double relative_deg = 0.0;
    double target_deg = 0.0;
    bool use_relative = false;
    bool use_target = false;
    double current_deg = 0.0;
    bool current_set = false;
    bool show_pins = false;
    bool use_ramp = false;

    // Ramp parameters with defaults
    double max_speed = 45.0;
    double min_speed = 5.0;
    double accel = 30.0;
    double decel = 30.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto need_value = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) {
                usage(argv[0]);
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg == "--degrees") {
            if (!parse_double(need_value(arg), relative_deg)) {
                std::cerr << "Invalid degrees value\n";
                return 1;
            }
            use_relative = true;
        } else if (arg == "--to") {
            if (!parse_double(need_value(arg), target_deg)) {
                std::cerr << "Invalid target degrees\n";
                return 1;
            }
            use_target = true;
        } else if (arg == "--current") {
            if (!parse_double(need_value(arg), current_deg)) {
                std::cerr << "Invalid current degrees\n";
                return 1;
            }
            current_set = true;
        } else if (arg == "--pins") {
            show_pins = true;
        } else if (arg == "--ramp") {
            use_ramp = true;
        } else if (arg == "--max-speed") {
            if (!parse_double(need_value(arg), max_speed)) {
                std::cerr << "Invalid max-speed value\n";
                return 1;
            }
        } else if (arg == "--min-speed") {
            if (!parse_double(need_value(arg), min_speed)) {
                std::cerr << "Invalid min-speed value\n";
                return 1;
            }
        } else if (arg == "--accel") {
            if (!parse_double(need_value(arg), accel)) {
                std::cerr << "Invalid accel value\n";
                return 1;
            }
        } else if (arg == "--decel") {
            if (!parse_double(need_value(arg), decel)) {
                std::cerr << "Invalid decel value\n";
                return 1;
            }
        } else if (arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    if (show_pins) {
        print_pins();
        return 0;
    }

    if (use_relative == use_target) {  // both false or both true
        usage(argv[0]);
        return 1;
    }

    if (use_ramp && use_relative) {
        std::cerr << "Error: --ramp can only be used with --to, not --degrees\n";
        return 1;
    }

    try {
        TMC2209::Pins pins{
            STEP_PIN,
            DIR_PIN,
            ENABLE_PIN,
            ENABLE_ACTIVE_HIGH != 0,
            MS1_PRESENT != 0,
            MS1_PIN,
            MS2_PRESENT != 0,
            MS2_PIN,
        };

        DomeMotor dome(GPIO_CHIP,
                       pins,
                       static_cast<double>(MOTOR_FULL_STEPS),
                       static_cast<double>(GEAR_RATIO),
                       MS1_HIGH != 0,
                       MS2_HIGH != 0);

        if (current_set) {
            dome.setCurrentAngleDeg(current_deg);
        }

        dome.enable();
        if (use_relative) {
            std::cout << "Rotating dome by " << relative_deg << " degrees\n";
            const double steps_per_deg = dome.stepsPerDomeDeg();
            const std::uint32_t steps_needed =
                static_cast<std::uint32_t>(std::llround(std::abs(relative_deg) * steps_per_deg));
            std::cout << "Steps to move: " << steps_needed << "\n";
            dome.rotateDegrees(relative_deg);
        } else if (use_target && !use_ramp) {
            std::cout << "Rotating dome to " << target_deg << " degrees (simple move)\n";
            const double error = dome.shortestError(target_deg, dome.currentAngleDeg());
            const double steps_per_deg = dome.stepsPerDomeDeg();
            const std::uint32_t steps_needed =
                static_cast<std::uint32_t>(std::llround(std::abs(error) * steps_per_deg));
            std::cout << "Steps to move: " << steps_needed << "\n";
            dome.rotateToAngleOpenLoop(target_deg);
        } else if (use_target && use_ramp) {
            std::cout << "Rotating dome to " << target_deg << " degrees (with speed ramping)\n";
            const double error = dome.shortestError(target_deg, dome.currentAngleDeg());
            std::cout << "Distance to travel: " << std::abs(error) << " degrees\n";
            std::cout << "Ramp Parameters:\n";
            std::cout << "  Max speed: " << max_speed << " deg/s\n";
            std::cout << "  Min speed: " << min_speed << " deg/s\n";
            std::cout << "  Acceleration: " << accel << " deg/s^2\n";
            std::cout << "  Deceleration: " << decel << " deg/s^2\n";

            DomeMotor::RampConfig ramp_config(max_speed, min_speed, accel, decel);
            dome.rotateToAngleWithRamp(target_deg, ramp_config);

            std::cout << "Reached target angle: " << dome.currentAngleDeg() << " degrees\n";
        }
        dome.disable();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
