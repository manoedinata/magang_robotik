#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

class DecisionMakerNode : public rclcpp::Node
{
public:
    DecisionMakerNode() : Node("decision_maker_node") {}

    void init_publishers_and_subscribers()
    {
        // Publisher untuk data keputusan (perintah)
        decision_pub_ = this->create_publisher<std_msgs::msg::String>("decision", 10);

        // Subscribe ke telemetry (dari OpenCV)
        telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
            "vehicle/telemetry",
            10,
            std::bind(&DecisionMakerNode::telemetry_callback, this, std::placeholders::_1)
        );
        // Subscribe ke sensor (dari Bluetooth/ESP32)
        sensors_sub_ = this->create_subscription<std_msgs::msg::String>(
            "sensors",
            10,
            std::bind(&DecisionMakerNode::sensors_callback, this, std::placeholders::_1)
        );
        // Subscribe ke mode manual/auto
        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "manual_control",
            10,
            std::bind(&DecisionMakerNode::mode_callback, this, std::placeholders::_1)
        );

        // Buat timer untuk menjalankan loop keputusan
        // Akan memanggil make_decision() setiap 100ms (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DecisionMakerNode::make_decision, this)
        );

        RCLCPP_INFO(this->get_logger(), "Decision Maker Node started.");
    }

private:
    // ROS 2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensors_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Variabel untuk menyimpan data terakhir yang diterima
    std::mutex data_mutex_;
    float latest_turn_angle_ = 0.0;
    std::string latest_road_status_ = "Detected";
    float latest_distance_ = 100.0; // Default ke jarak aman
    bool manual_control_ = true;

    // Callbacks
    void mode_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        manual_control_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Manual control mode: %s", manual_control_ ? "ENABLED" : "DISABLED");
    }

    void telemetry_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
    {
        bool is_manual;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            is_manual = manual_control_;
        }
        if(is_manual) return; // Abaikan jika dalam mode manual

        try {
            // Parse JSON telemetry
            auto telemetry_json = nlohmann::json::parse(msg->data);

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_turn_angle_ = telemetry_json["turn_angle"];
                latest_road_status_ = telemetry_json["road_status"];
            }
        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse telemetry JSON: %s", e.what());
        }
    }

    void sensors_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
    {
        bool is_manual;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            is_manual = manual_control_;
        }
        if(is_manual) return; // Abaikan jika dalam mode manual

        try {
            // Parse JSON sensor data
            auto sensors_json = nlohmann::json::parse(msg->data);

            if (sensors_json["command"] != "telemetry") {
                RCLCPP_WARN(this->get_logger(), "Ignoring non-telemetry message.");
                return;
            }

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_distance_ = sensors_json["distance"];
            }

        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse sensors JSON: %s", e.what());
        }
    }

    // Implementasi logika keputusan
    void make_decision()
    {
        bool is_manual;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            is_manual = manual_control_;
        }
        if(is_manual) return; // Abaikan jika dalam mode manual

        // Buat salinan (copy) data yang aman dari thread
        float current_angle;
        std::string current_status;
        float current_distance;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_angle = latest_turn_angle_;
            current_status = latest_road_status_;
            current_distance = latest_distance_;
        }

        // Logika keputusan
        float target_speed = 1.0; // Kecepatan dasar (misal 1.0 m/s)
        float target_steer = current_angle;

        // Ada halangan! BERHENTI.
        if (current_distance < 30.0) { // Jarak dalam cm
            target_speed = 0.0;
            RCLCPP_WARN(this->get_logger(), "OBSTACLE DETECTED! Stopping.");
        }
        
        // Kehilangan jalur! BERHENTI.
        else if (current_status != "Detected") {
            target_speed = 0.0;
            RCLCPP_WARN(this->get_logger(), "LANE LOST! Stopping.");
        }

        // Buat perintah dalam format JSON
        nlohmann::json decision_json;
        decision_json["type"] = "command";
        decision_json["speed"] = target_speed;
        decision_json["turn_angle"] = target_steer;

        // Publikasikan perintah sebagai string
        auto msg = std_msgs::msg::String();
        msg.data = decision_json.dump();
        decision_pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Published decision: %s", msg.data.c_str());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DecisionMakerNode>();
    node->init_publishers_and_subscribers();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
