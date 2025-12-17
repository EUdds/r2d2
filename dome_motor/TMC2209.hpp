#pragma once

#include <chrono>
#include <cstdint>
#include <string>

struct gpiod_chip;
struct gpiod_line;

// Minimal step/dir driver wrapper for a TMC2209 breakout using libgpiod.
class TMC2209 {
public:
    struct Pins {
        unsigned int step;
        unsigned int dir;
        unsigned int enable;
        // Most TMC2209 boards use active-low enable; set to true if yours is active-high.
        bool enable_active_high = false;
        // Optional microstep select lines.
        bool has_ms1 = false;
        unsigned int ms1 = 0;
        bool has_ms2 = false;
        unsigned int ms2 = 0;
    };

    TMC2209(const std::string& chip_name, Pins pins);
    ~TMC2209();

    TMC2209(const TMC2209&) = delete;
    TMC2209& operator=(const TMC2209&) = delete;
    TMC2209(TMC2209&&) = delete;
    TMC2209& operator=(TMC2209&&) = delete;

    void enable();
    void disable();
    void setDirection(bool clockwise);
    void setMicrostepPins(bool ms1_high, bool ms2_high);
    void stepOnce(std::chrono::microseconds pulse_width = std::chrono::microseconds{4});
    void step(std::uint32_t count,
              std::chrono::microseconds pulse_width = std::chrono::microseconds{4},
              std::chrono::microseconds step_delay = std::chrono::microseconds{500});

private:
    gpiod_chip* chip_;
    gpiod_line* step_line_;
    gpiod_line* dir_line_;
    gpiod_line* en_line_;
    gpiod_line* ms1_line_;
    gpiod_line* ms2_line_;
    bool enable_active_high_;

    void requestOutputLine(unsigned int offset, int default_value, const char* label, gpiod_line** out_line);
    void setLineValue(gpiod_line* line, int value, const char* label);
};
