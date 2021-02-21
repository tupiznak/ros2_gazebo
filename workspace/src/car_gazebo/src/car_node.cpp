#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "car_msgs/msg/kinematic.hpp"
#include "car_msgs/msg/airbag.hpp"

# include "car_node.h"

using namespace std::chrono_literals;

Car::Car()
    : Node("car_gazebo")
{
    sensor_pub = this->create_publisher<car_msgs::msg::Kinematic>("/sensor/kinematic", 1);
    command_sub = this->create_subscription<car_msgs::msg::Airbag>(
                "/command/airbag", 10, std::bind(&Car::command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
                1000ms, std::bind(&Car::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Симуляционная нода запущена");
}

void Car::timer_callback() {
    auto message = car_msgs::msg::Kinematic();
    message.acceleration = 12;
    RCLCPP_INFO(this->get_logger(), "Текущее ускорение: '%f'", message.acceleration);
    sensor_pub->publish(message);
}

void Car::command_callback(const car_msgs::msg::Airbag::SharedPtr msg) const {
    if (msg->drop)
        RCLCPP_WARN(this->get_logger(), "Получена команда на выброс");
    else
        RCLCPP_INFO(this->get_logger(), "Получена команда");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Car>());
    rclcpp::shutdown();
    return 0;
}
