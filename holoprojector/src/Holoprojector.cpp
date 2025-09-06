#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <random>

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
        
        auto topic_callback = [this](const std_msgs::msg::String::UniquePtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        };

        auto subscription = this->create_subscription<std_msgs::msg::String>(
            "debug", 10, topic_callback);
        
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

    void apply(void)
    {
        // Apply servo angles and LED colors
        RCLCPP_INFO(this->get_logger(), "Applying servo1: %.2f, servo2: %.2f", servo1_angle_, servo2_angle_);
        cmd_.set_servo1_angle(servo1_angle_);
        cmd_.set_servo2_angle(servo2_angle_);
        for (int i = 0; i < NEOPIXEL_COUNT; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "LED %d - R: %d, G: %d, B: %d", i, led_cmds_[i].r(), led_cmds_[i].g(), led_cmds_[i].b());
            auto* led_cmd = cmd_.add_led_commands();
            led_cmd->set_r(led_cmds_[i].r());
            led_cmd->set_g(led_cmds_[i].g());
            led_cmd->set_b(led_cmds_[i].b());
            led_cmd->set_brightness(led_cmds_[i].brightness());
        }
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

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoloprojectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}