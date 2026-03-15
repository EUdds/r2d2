#include "TMC2209.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

struct Options {
    std::string chip = "gpiochip0";
    unsigned int step = std::numeric_limits<unsigned int>::max();
    unsigned int dir = std::numeric_limits<unsigned int>::max();
    unsigned int enable = std::numeric_limits<unsigned int>::max();
    bool enable_active_high = false;
    bool clockwise = true;
    std::uint32_t steps = 200;
    std::chrono::microseconds pulse = std::chrono::microseconds{5};
    std::chrono::microseconds delay = std::chrono::microseconds{500};
    bool keep_enabled = false;
};

void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " --chip <chip> --step <offset> --dir <offset> --enable <offset> [options]\n"
              << "Options:\n"
              << "  --chip <name>           gpiochip name (default gpiochip0)\n"
              << "  --step <offset>         step GPIO line offset (required)\n"
              << "  --dir <offset>          direction GPIO line offset (required)\n"
              << "  --enable <offset>       enable GPIO line offset (required)\n"
              << "  --enable-active-high    treat enable as active-high (default active-low)\n"
              << "  --cw / --ccw            direction (default cw)\n"
              << "  --steps <count>         step count (default 200)\n"
              << "  --pulse-us <microsec>   step pulse width (default 5)\n"
              << "  --delay-us <microsec>   delay between steps (default 500)\n"
              << "  --keep-enabled          do not disable driver after motion\n"
              << "  --help                  show this help\n";
}

bool parse_uint(const char* text, unsigned int& out) {
    try {
        out = static_cast<unsigned int>(std::stoul(text));
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_u32(const char* text, std::uint32_t& out) {
    try {
        out = static_cast<std::uint32_t>(std::stoul(text));
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options opts;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " requires a value\n";
                print_usage(argv[0]);
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--chip") {
            opts.chip = require_value(arg);
        } else if (arg == "--step") {
            if (!parse_uint(require_value(arg), opts.step)) {
                std::cerr << "Invalid step offset\n";
                return 1;
            }
        } else if (arg == "--dir") {
            if (!parse_uint(require_value(arg), opts.dir)) {
                std::cerr << "Invalid dir offset\n";
                return 1;
            }
        } else if (arg == "--enable") {
            if (!parse_uint(require_value(arg), opts.enable)) {
                std::cerr << "Invalid enable offset\n";
                return 1;
            }
        } else if (arg == "--enable-active-high") {
            opts.enable_active_high = true;
        } else if (arg == "--cw") {
            opts.clockwise = true;
        } else if (arg == "--ccw") {
            opts.clockwise = false;
        } else if (arg == "--steps") {
            if (!parse_u32(require_value(arg), opts.steps)) {
                std::cerr << "Invalid step count\n";
                return 1;
            }
        } else if (arg == "--pulse-us") {
            unsigned int value = 0;
            if (!parse_uint(require_value(arg), value)) {
                std::cerr << "Invalid pulse width\n";
                return 1;
            }
            opts.pulse = std::chrono::microseconds{value};
        } else if (arg == "--delay-us") {
            unsigned int value = 0;
            if (!parse_uint(require_value(arg), value)) {
                std::cerr << "Invalid step delay\n";
                return 1;
            }
            opts.delay = std::chrono::microseconds{value};
        } else if (arg == "--keep-enabled") {
            opts.keep_enabled = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    if (opts.step == std::numeric_limits<unsigned int>::max() ||
        opts.dir == std::numeric_limits<unsigned int>::max() ||
        opts.enable == std::numeric_limits<unsigned int>::max()) {
        std::cerr << "Missing required GPIO offsets\n";
        print_usage(argv[0]);
        return 1;
    }

    try {
        TMC2209 driver(opts.chip, {opts.step, opts.dir, opts.enable, opts.enable_active_high});
        driver.enable();
        driver.setDirection(opts.clockwise);

        std::cout << "Stepping " << opts.steps << " steps "
                  << (opts.clockwise ? "CW" : "CCW") << " on " << opts.chip << "\n";
        driver.step(opts.steps, opts.pulse, opts.delay);
        if (!opts.keep_enabled) {
            driver.disable();
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
