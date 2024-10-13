#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

class BatteryStatePublisher : public rclcpp::Node
{
public:
    BatteryStatePublisher()
        : Node("battery_state_publisher"), battery_level_(0.5) // Start with 50% battery
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("robot1/battery_state", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&BatteryStatePublisher::publish_battery_state, this));
        RCLCPP_INFO(this->get_logger(), "Battery State Publisher Node has been started.");
    }

private:
    void publish_battery_state()
    {
        auto message = sensor_msgs::msg::BatteryState();
        message.header.stamp = this->get_clock()->now();

        message.voltage = 12.0;       // Voltage in volts
        message.current = -1.5;       // Current in amps (negative for discharging)
        message.charge = 50.0;        // Charge in Amp-hours
        message.capacity = 100.0;     // Total capacity in Amp-hours
        message.design_capacity = 100.0; // Design capacity in Amp-hours
        message.percentage = battery_level_; // Battery percentage (0.0 - 1.0)
        message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
        message.present = true;

        // Publish the message
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published Battery State: %.2f%%", battery_level_ * 100);
    }

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double battery_level_; // Battery percentage (0.0 - 1.0)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStatePublisher>());
    rclcpp::shutdown();
    return 0;
}

