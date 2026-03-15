#include "TMC2209.hpp"

#include <gpiod.h>

#include <stdexcept>
#include <thread>

TMC2209::TMC2209(const std::string& chip_name, Pins pins)
    : chip_(nullptr),
      step_line_(nullptr),
      dir_line_(nullptr),
      en_line_(nullptr),
      ms1_line_(nullptr),
      ms2_line_(nullptr),
      enable_active_high_(pins.enable_active_high) {
    chip_ = gpiod_chip_open_by_name(chip_name.c_str());
    if (!chip_) {
        throw std::runtime_error("Failed to open GPIO chip " + chip_name);
    }

    try {
        requestOutputLine(pins.step, 0, "tmc2209-step", &step_line_);
        requestOutputLine(pins.dir, 0, "tmc2209-dir", &dir_line_);
        const int disable_value = enable_active_high_ ? 0 : 1;
        requestOutputLine(pins.enable, disable_value, "tmc2209-enable", &en_line_);
        if (pins.has_ms1) {
            requestOutputLine(pins.ms1, 0, "tmc2209-ms1", &ms1_line_);
        }
        if (pins.has_ms2) {
            requestOutputLine(pins.ms2, 0, "tmc2209-ms2", &ms2_line_);
        }
    } catch (...) {
        if (step_line_) {
            gpiod_line_release(step_line_);
        }
        if (dir_line_) {
            gpiod_line_release(dir_line_);
        }
        if (en_line_) {
            gpiod_line_release(en_line_);
        }
        if (ms1_line_) {
            gpiod_line_release(ms1_line_);
        }
        if (ms2_line_) {
            gpiod_line_release(ms2_line_);
        }
        gpiod_chip_close(chip_);
        throw;
    }
}

TMC2209::~TMC2209() {
    if (step_line_) {
        gpiod_line_release(step_line_);
    }
    if (dir_line_) {
        gpiod_line_release(dir_line_);
    }
    if (en_line_) {
        gpiod_line_release(en_line_);
    }
    if (ms1_line_) {
        gpiod_line_release(ms1_line_);
    }
    if (ms2_line_) {
        gpiod_line_release(ms2_line_);
    }
    if (chip_) {
        gpiod_chip_close(chip_);
    }
}

void TMC2209::enable() {
    const int enable_value = enable_active_high_ ? 1 : 0;
    setLineValue(en_line_, enable_value, "enable");
}

void TMC2209::disable() {
    const int disable_value = enable_active_high_ ? 0 : 1;
    setLineValue(en_line_, disable_value, "enable");
}

void TMC2209::setDirection(bool clockwise) {
    setLineValue(dir_line_, clockwise ? 1 : 0, "dir");
}

void TMC2209::setMicrostepPins(bool ms1_high, bool ms2_high) {
    if (!ms1_line_ && !ms2_line_) {
        throw std::runtime_error("Microstep pins not configured on TMC2209");
    }
    if (ms1_line_) {
        setLineValue(ms1_line_, ms1_high ? 1 : 0, "ms1");
    } else if (ms1_high) {
        throw std::runtime_error("MS1 pin not configured but requested high");
    }
    if (ms2_line_) {
        setLineValue(ms2_line_, ms2_high ? 1 : 0, "ms2");
    } else if (ms2_high) {
        throw std::runtime_error("MS2 pin not configured but requested high");
    }
}

void TMC2209::stepOnce(std::chrono::microseconds pulse_width) {
    setLineValue(step_line_, 1, "step");
    std::this_thread::sleep_for(pulse_width);
    setLineValue(step_line_, 0, "step");
}

void TMC2209::step(std::uint32_t count,
                   std::chrono::microseconds pulse_width,
                   std::chrono::microseconds step_delay) {
    for (std::uint32_t i = 0; i < count; ++i) {
        stepOnce(pulse_width);
        if (i + 1 < count) {
            std::this_thread::sleep_for(step_delay);
        }
    }
}

void TMC2209::requestOutputLine(unsigned int offset,
                                int default_value,
                                const char* label,
                                gpiod_line** out_line) {
    *out_line = gpiod_chip_get_line(chip_, offset);
    if (!*out_line) {
        throw std::runtime_error(std::string("Failed to get GPIO line for ") + label);
    }
    if (gpiod_line_request_output(*out_line, label, default_value) < 0) {
        *out_line = nullptr;
        throw std::runtime_error(std::string("Failed to request GPIO line for ") + label);
    }
}

void TMC2209::setLineValue(gpiod_line* line, int value, const char* label) {
    if (!line) {
        throw std::runtime_error(std::string("GPIO line for ") + label + " is not initialized");
    }
    if (gpiod_line_set_value(line, value) < 0) {
        throw std::runtime_error(std::string("Failed to set GPIO line for ") + label);
    }
}
