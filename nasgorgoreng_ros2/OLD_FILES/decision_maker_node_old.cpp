#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#include "std_msgs/msg/float32.hpp" // Float32
#include "std_msgs/msg/string.hpp"  // String

class DecisionMakerNode : public rclcpp::Node
{
public:
    DecisionMakerNode() : Node("decision_maker_node") {}

    void init_publishers_and_subscribers()
    {
        // Publisher untuk data telemetri
        decision_pub_ = this->create_publisher<std_msgs::msg::String>("decision", 10);

        // Subscribe ke telemetry
        telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
            "vehicle/telemetry",
            10,
            std::bind(&DecisionMakerNode::telemetry_callback, this, std::placeholders::_1)
        );
        // Subscribe ke sensor
        sensors_sub_ = this->create_subscription<std_msgs::msg::String>(
            "sensors",
            10,
            std::bind(&DecisionMakerNode::sensors_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Lane Detector Node started. Subscribing to /camera/image_raw");
    }

private:
    // ROS 2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensors_sub_;

    void telemetry_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // Parse JSON telemetry
        auto telemetry_json = nlohmann::json::parse(msg->data);

        float turn_angle = telemetry_json["turn_angle"];
        std::string road_status = telemetry_json["status"];
    }

    void sensors_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // Parse JSON sensor data
        auto sensors_json = nlohmann::json::parse(msg->data);

        float distance = sensors_json["distance"];
    }

    void make_decision()
    {
        // TODO
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DecisionMakerNode>();
    node->init_publishers_and_subscribers();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
