#ifndef CAR_NODE_H
#define CAR_NODE_H

class Car : public rclcpp::Node
{
public:
    Car();

private:
    void timer_callback();
    void command_callback(const car_msgs::msg::Airbag::SharedPtr msg) const;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::Kinematic>::SharedPtr sensor_pub;
    rclcpp::Subscription<car_msgs::msg::Airbag>::SharedPtr command_sub;
};
#endif // CAR_NODE_H
