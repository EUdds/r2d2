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
#define STEP_PIN 3
#endif

#ifndef DIR_PIN
#define DIR_PIN 2
#endif

#ifndef ENABLE_PIN
#define ENABLE_PIN 22
#endif

#ifndef ENABLE_ACTIVE_HIGH
#define ENABLE_ACTIVE_HIGH 0
#endif

#ifndef MS1_PRESENT
#define MS1_PRESENT 1
#endif

#ifndef MS1_PIN
#define MS1_PIN 4
#endif

#ifndef MS2_PRESENT
#define MS2_PRESENT 1
#endif

#ifndef MS2_PIN
#define MS2_PIN 17
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
    std::cerr << "Usage: " << prog << " (--degrees <deg> | --to <deg>) [--current <deg>]\n"
              << "Pins and microstep mode are set at compile time via -D macros (use --copt to override).\n";
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
        } else if (arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    if (use_relative == use_target) {  // both false or both true
        usage(argv[0]);
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
        } else if (use_target) {
            std::cout << "Rotating dome to " << target_deg << " degrees (open-loop)\n";
            const double error = dome.shortestError(target_deg, dome.currentAngleDeg());
            const double steps_per_deg = dome.stepsPerDomeDeg();
            const std::uint32_t steps_needed =
                static_cast<std::uint32_t>(std::llround(std::abs(error) * steps_per_deg));
            std::cout << "Steps to move: " << steps_needed << "\n";
            dome.rotateToAngleOpenLoop(target_deg);
        }
        dome.disable();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
