// Send telemetry data to ESP32 via Bluetooth
#include <iostream>
#include <thread>
#include <simpleble/SimpleBLE.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#define ESP32_NAME "nasgorgoreng_esp32"
#define SERVICE_UUID "e812cc92-1296-49a3-932a-a0a871ca8aaf"
#define CHARACTERISTIC_UUID "21184893-98f5-4229-83a4-c7259b8fd0bc"

class PublishBTESP32Node : public rclcpp::Node
{
public:
    PublishBTESP32Node() : Node("publish_bt_esp32_node") {}

    void init()
    {
        // Subscribe to turn combined topic
        decision_sub_ = this->create_subscription<std_msgs::msg::String>(
            "decision",
            10,
            std::bind(&PublishBTESP32Node::topic_callback, this, std::placeholders::_1)
        );

        // Setup Bluetooth
        std::thread([this]() { this->init_ble(); }).detach();

        RCLCPP_INFO(this->get_logger(), "Publish Bluetooth data to ESP32 Node started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_sub_;
    std::shared_ptr<SimpleBLE::Peripheral> esp32_;
    std::string service_uuid = "e812cc92-1296-49a3-932a-a0a871ca8aaf";
    std::string characteristic_uuid = PublishBTESP32Node::service_uuid;
    bool connected_ = false;


    void init_ble() {
        RCLCPP_INFO(this->get_logger(), "Scanning for BLE devices...");

        std::vector<SimpleBLE::Adapter> adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No BLE adapters found!");
            return;
        }

        SimpleBLE::Adapter adapter = adapters[0];
        RCLCPP_INFO(this->get_logger(), "Using adapter: %s", adapter.identifier().c_str());
        adapter.power_on();

        // Check if already connected to ESP32
        std::vector<SimpleBLE::Peripheral> paired_peripherals = adapter.get_paired_peripherals();
        for (auto peripheral : paired_peripherals) {
            if (peripheral.identifier() == ESP32_NAME) {
                esp32_ = std::make_shared<SimpleBLE::Peripheral>(peripheral);
                break;
            }
        }

        // Scan for devices
        while (!esp32_) {
            RCLCPP_ERROR(this->get_logger(), "ESP32 not found! Retrying...");
            adapter.scan_for(4000); // 4s scan

            std::vector<SimpleBLE::Peripheral> peripherals = adapter.scan_get_results();
            RCLCPP_INFO(this->get_logger(), "Found %zu devices", peripherals.size());

            for (auto peripheral : peripherals) {
                if (peripheral.identifier() == ESP32_NAME) {
                    esp32_ = std::make_shared<SimpleBLE::Peripheral>(peripheral);
                    break;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Found ESP32: %s", esp32_->identifier().c_str());

        // Connect to ESP32
        RCLCPP_INFO(this->get_logger(), "Connecting to ESP32...");
        esp32_->connect();

        if (esp32_->is_connected()) {
            connected_ = true;
            RCLCPP_INFO(this->get_logger(), "Connected to ESP32!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to ESP32.");
        }
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!connected_ || !esp32_ || !esp32_->is_connected()) {
            RCLCPP_WARN(this->get_logger(), "ESP32 not connected yet. Message ignored.");
            return;
        }

        try {
            esp32_->write_request((std::string)CHARACTERISTIC_UUID, (std::string)SERVICE_UUID, msg->data);

            RCLCPP_INFO(this->get_logger(), "Sent BLE Data: %s", msg->data.c_str());
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending: %s", e.what());
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PublishBTESP32Node>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

