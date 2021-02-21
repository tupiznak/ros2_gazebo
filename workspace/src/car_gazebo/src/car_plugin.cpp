#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "car_msgs/msg/kinematic.hpp"
#include "car_msgs/msg/airbag.hpp"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

using namespace std::chrono_literals;

namespace gazebo {

class CarPlugin : public ModelPlugin
{
public:
    CarPlugin()
        : ModelPlugin()
    {
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("car_gazebo");

        sensor_pub = node->create_publisher<car_msgs::msg::Kinematic>("/sensor/kinematic", 1);
        command_sub = node->create_subscription<car_msgs::msg::Airbag>(
                    "/command/airbag", 10, std::bind(&CarPlugin::command_callback, this, std::placeholders::_1));

        timer_ = node->create_wall_timer(
                    std::chrono_literals::operator""s(_time_delta), std::bind(&CarPlugin::timer_callback, this));

        std::thread executor([this](){rclcpp::spin(node);});
        executor.detach();

        this->model = _parent;

        gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        gazebo_node->Init(model->GetWorld()->Name());
        pub_visual = gazebo_node->Advertise<gazebo::msgs::Visual>("~/visual");

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CarPlugin::OnUpdate, this));

        RCLCPP_INFO(node->get_logger(), "Симуляционная нода запущена");
    }

    public: void OnUpdate()
    {
      auto car = model->GetLink("car::car_box");
      car->SetForce(ignition::math::Vector3d(20, 0, 0));
    }

    private: physics::ModelPtr model;

    private: event::ConnectionPtr updateConnection;

private:

    void timer_callback() {

        auto current_speed = this->model->RelativeLinearVel().X();
        auto acceleration = (current_speed - _last_speed) / _time_delta;
        _last_speed = current_speed;

        auto message = car_msgs::msg::Kinematic();
        message.acceleration = std::abs(acceleration);
        RCLCPP_INFO(node->get_logger(), "Текущее ускорение: '%f'", message.acceleration);
        sensor_pub->publish(message);
    }
    void command_callback(const car_msgs::msg::Airbag::SharedPtr msg) {
        if (msg->drop) {

            auto airbag = model->GetLink("car::link_airbag");
            auto visualMsg = airbag->GetVisualMessage("airbag_visual");

            visualMsg.set_name(airbag->GetScopedName());
            visualMsg.set_parent_name(model->GetScopedName());
            visualMsg.set_allocated_scale(
                        new msgs::Vector3d(
                            msgs::Convert(ignition::math::Vector3d(1.5, 1.5, 1.5))));

            pub_visual->Publish(visualMsg);

            RCLCPP_WARN(node->get_logger(), "Получена команда на выброс");
        }
        else
            RCLCPP_INFO(node->get_logger(), "Получена команда");
    }

    rclcpp::Node::SharedPtr node;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::Kinematic>::SharedPtr sensor_pub;
    rclcpp::Subscription<car_msgs::msg::Airbag>::SharedPtr command_sub;

    gazebo::transport::PublisherPtr pub_visual;
    gazebo::transport::NodePtr gazebo_node;

    double _last_speed{};
    double _time_delta = 0.1;

};
GZ_REGISTER_MODEL_PLUGIN(CarPlugin)

}
