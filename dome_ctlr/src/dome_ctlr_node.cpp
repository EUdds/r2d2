#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "dome_motor/DomeMotor.hpp"

using namespace std::chrono_literals;

class DomeControllerNode : public rclcpp::Node
{
public:
  DomeControllerNode() : Node("dome_ctlr")
  {
    // Hardware and motor parameters (mirrors dome_cli defaults; override via params).
    chip_name_ = declare_parameter<std::string>("chip_name", "gpiochip4");
    pins_.step = declare_parameter<int>("step_pin", 3);
    pins_.dir = declare_parameter<int>("dir_pin", 2);
    pins_.enable = declare_parameter<int>("enable_pin", 22);
    pins_.enable_active_high = declare_parameter<bool>("enable_active_high", false);
    pins_.has_ms1 = declare_parameter<bool>("ms1_present", true);
    pins_.ms1 = declare_parameter<int>("ms1_pin", 4);
    pins_.has_ms2 = declare_parameter<bool>("ms2_present", true);
    pins_.ms2 = declare_parameter<int>("ms2_pin", 17);

    const bool ms1_high = declare_parameter<bool>("ms1_high", false);
    const bool ms2_high = declare_parameter<bool>("ms2_high", false);
    const double motor_full_steps = declare_parameter<double>("motor_full_steps", 200.0);
    const double gear_ratio = declare_parameter<double>("gear_ratio", 124.0 / 15.0);
    const int pulse_width_us = declare_parameter<int>("pulse_width_us", 5);
    const int min_step_delay_us = declare_parameter<int>("min_step_delay_us", 500);
    const double initial_angle = declare_parameter<double>("current_angle_deg", 0.0);
    const bool enable_on_start = declare_parameter<bool>("enable_on_start", true);
    seek_topic_ = declare_parameter<std::string>("seek_topic", "dome/seek");
    use_seek_topic_ = declare_parameter<bool>("use_seek_topic", false);
    speed_topic_ = declare_parameter<std::string>("speed_topic", "dome/speed");
    const int speed_timer_period_ms = declare_parameter<int>("speed_timer_period_ms", 20);
    speed_command_timeout_ = std::chrono::milliseconds(
      declare_parameter<int>("speed_command_timeout_ms", 200));
    max_dt_ = std::chrono::duration<double>(
      declare_parameter<double>("max_dt_seconds", 0.1));

    DomeMotor::TimingConfig timing(
      std::chrono::microseconds{pulse_width_us},
      std::chrono::microseconds{min_step_delay_us});

    try {
      dome_motor_ = std::make_unique<DomeMotor>(
        chip_name_,
        pins_,
        motor_full_steps,
        gear_ratio,
        ms1_high,
        ms2_high,
        timing);
      dome_motor_->setCurrentAngleDeg(initial_angle);
      if (enable_on_start) {
        dome_motor_->enable();
      }
    } catch (const std::exception &ex) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize DomeMotor: %s", ex.what());
      throw;
    }

    if (use_seek_topic_) {
      seek_sub_ = create_subscription<std_msgs::msg::Float64>(
        seek_topic_, rclcpp::QoS(10),
        std::bind(&DomeControllerNode::handle_seek, this, std::placeholders::_1));
    }

    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      speed_topic_, rclcpp::QoS(10),
      std::bind(&DomeControllerNode::handle_speed, this, std::placeholders::_1));

    const auto now_time = now();
    last_speed_time_ = now_time;
    last_speed_msg_time_ = now_time;
    speed_timer_ = create_wall_timer(
      std::chrono::milliseconds(speed_timer_period_ms),
      std::bind(&DomeControllerNode::speed_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "dome_ctlr initialized. Listening for angle on '%s' (enabled=%s) and speed on '%s'.",
      seek_topic_.c_str(),
      use_seek_topic_ ? "true" : "false",
      speed_topic_.c_str());
  }

  ~DomeControllerNode() override
  {
    if (dome_motor_) {
      dome_motor_->disable();
    }
  }

private:
  void handle_seek(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!dome_motor_) {
      return;
    }
    const double target = msg->data;
    dome_motor_->rotateToAngleOpenLoop(target);
  }

  void handle_speed(const std_msgs::msg::Float64::SharedPtr msg)
  {
    last_speed_command_ = msg->data;
    last_speed_msg_time_ = now();
  }

  void speed_tick()
  {
    if (!dome_motor_) {
      return;
    }
    const auto now_time = now();
    const double dt = last_speed_time_.nanoseconds() > 0
                        ? (now_time - last_speed_time_).seconds()
                        : 0.0;
    last_speed_time_ = now_time;
    if (dt <= 0.0) {
      return;
    }
    double commanded_speed = last_speed_command_;
    if ((now_time - last_speed_msg_time_) > speed_command_timeout_) {
      commanded_speed = 0.0;  // stop if command is stale
    }
    const double clamped_dt = std::min(dt, max_dt_.count());
    dome_motor_->seekAtSpeedFor(commanded_speed, std::chrono::duration<double>(clamped_dt));
  }

  std::unique_ptr<DomeMotor> dome_motor_;
  TMC2209::Pins pins_;
  std::string chip_name_;
  std::string seek_topic_;
  std::string speed_topic_;
  bool use_seek_topic_{false};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr seek_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::TimerBase::SharedPtr speed_timer_;
  rclcpp::Time last_speed_time_;
  rclcpp::Time last_speed_msg_time_;
  double last_speed_command_{0.0};
  std::chrono::milliseconds speed_command_timeout_{200};
  std::chrono::duration<double> max_dt_{0.1};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DomeControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("dome_ctlr"), "dome_ctlr crashed during startup: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
