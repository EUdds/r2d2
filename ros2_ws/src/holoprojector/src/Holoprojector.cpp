#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <random>

#include "interfaces/msg/holoprojector_state.hpp"

#include "SerialInterface.hpp"
#include "holoprojector.pb.h"

using namespace std::chrono_literals;

constexpr int NEOPIXEL_COUNT = 7;

class HoloprojectorNode : public rclcpp::Node
{
public:
    HoloprojectorNode() : Node("holoprojector_node")
    {
        timer_ = this->create_wall_timer(
            500ms, std::bind(&HoloprojectorNode::timer_callback, this));

        timer10ms_ = this->create_wall_timer(
            10ms, std::bind(&HoloprojectorNode::task_callback_10ms, this));
        

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
        auto topic_callback = [this](const interfaces::msg::HoloprojectorState msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "Received HoloprojectorState message");
            set_servo_angle(1, msg.servo1_angle);
            set_servo_angle(2, msg.servo2_angle);
            fill_color(msg.led_r, msg.led_g, msg.led_b);
            apply();
        };

        subscription_ = this->create_subscription<interfaces::msg::HoloprojectorState>(
            "debug", qos, topic_callback);
        
        if (serial_interface_.open())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        }
    }

private:
    void timer_callback()
    {
    }

    void task_callback_10ms()
    {
        uint8_t buffer[256];
        size_t available_bytes = serial_interface_.available();
        if (available_bytes > 0)
        {
            int bytes_read = serial_interface_.readData(buffer, std::min(available_bytes, sizeof(buffer)));
            if (bytes_read > 0)
            {
                std::string received_data(reinterpret_cast<char*>(buffer), bytes_read);
                RCLCPP_INFO(this->get_logger(), "Received: %s", received_data.c_str());
            }
        }
    }

    void set_led(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
    {
        if (index < NEOPIXEL_COUNT)
        {
            led_cmds_[index].set_r(r);
            led_cmds_[index].set_g(g);
            led_cmds_[index].set_b(b);
            led_cmds_[index].set_brightness(128);
        }
    }

    void fill_color(uint8_t r, uint8_t g, uint8_t b)
    {
        for (int i = 0; i < NEOPIXEL_COUNT; ++i)
        {
            set_led(i, r, g, b);
        }
    }

    void set_servo_angle(uint8_t servo_index, float angle)
    {
        if (servo_index == 1)
        {
            servo1_angle_ = angle;
        }
        else if (servo_index == 2)
        {
            servo2_angle_ = angle;
        }
    }

    void send_(void)
    {
        auto crc16_ccitt = [](const uint8_t* data, size_t len) -> uint16_t {
            uint16_t crc = 0xFFFF;
            for (size_t i = 0; i < len; ++i) {
                crc ^= (uint16_t)data[i] << 8;
                for (int j = 0; j < 8; ++j) {
                    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                    else crc <<= 1;
                }
            }
            return crc;
        };

        std::string out;
        if (cmd_.SerializeToString(&out))
        {
            const uint8_t preamble[2] = {0xAA, 0x55};
            uint16_t len = static_cast<uint16_t>(out.size());
            uint8_t len_bytes[2] = {static_cast<uint8_t>(len & 0xFF), static_cast<uint8_t>((len >> 8) & 0xFF)};
            uint16_t crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(out.data()), out.size());
            uint8_t crc_bytes[2] = {static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>((crc >> 8) & 0xFF)};

            serial_interface_.writeData(preamble, sizeof(preamble));
            serial_interface_.writeData(len_bytes, sizeof(len_bytes));
            if (len > 0) {
                serial_interface_.writeData(reinterpret_cast<const uint8_t*>(out.data()), out.size());
            }
            serial_interface_.writeData(crc_bytes, sizeof(crc_bytes));

            RCLCPP_INFO(this->get_logger(), "Sent framed command, payload: %zu bytes", out.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to serialize command.");
        }
    }

    void apply(void)
    {
        // Apply servo angles and LED colors
        RCLCPP_INFO(this->get_logger(), "Applying servo1: %.2f, servo2: %.2f", servo1_angle_, servo2_angle_);
        cmd_.set_servo1_angle(servo1_angle_);
        cmd_.set_servo2_angle(servo2_angle_);
        cmd_.clear_led_commands();
        for (int i = 0; i < NEOPIXEL_COUNT; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "LED %d - R: %d, G: %d, B: %d", i, led_cmds_[i].r(), led_cmds_[i].g(), led_cmds_[i].b());
            auto* led_cmd = cmd_.add_led_commands();
            led_cmd->set_r(led_cmds_[i].r());
            led_cmd->set_g(led_cmds_[i].g());
            led_cmd->set_b(led_cmds_[i].b());
            led_cmd->set_brightness(led_cmds_[i].brightness());
        }
        send_();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer10ms_;
    SerialInterface serial_interface_{"/dev/ttyACM0", 115200};
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    holoprojector::HoloprojectorCommand cmd_;
    holoprojector::LEDCommand led_cmds_[NEOPIXEL_COUNT];
    float servo1_angle_ = 0.0f;
    float servo2_angle_ = 0.0f;
    rclcpp::Subscription<interfaces::msg::HoloprojectorState>::SharedPtr subscription_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoloprojectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
