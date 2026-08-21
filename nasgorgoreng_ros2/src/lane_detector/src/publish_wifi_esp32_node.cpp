// Send telemetry data to ESP8266 via Wi-Fi UDP
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// C/C++ Networking Libraries
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring> // For memset
#include <cerrno>  // For errno
#include <string>

// --- UDP Configuration ---
#define ESP8266_IP "10.42.0.79"
#define ESP8266_PORT 4210

class PublishUdpESP8266Node : public rclcpp::Node
{
public:
    PublishUdpESP8266Node() : Node("publish_udp_esp8266_node"), socket_fd_(-1)
    {
    }

    // Destructor to close the socket
    ~PublishUdpESP8266Node()
    {
        if (socket_fd_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Closing UDP socket.");
            close(socket_fd_);
        }
    }

    void init()
    {
        // Subscribe to the same "decision" topic
        decision_sub_ = this->create_subscription<std_msgs::msg::String>(
            "decision",
            10,
            std::bind(&PublishUdpESP8266Node::topic_callback, this, std::placeholders::_1));

        // Setup UDP socket
        init_udp();

        RCLCPP_INFO(this->get_logger(), "Publish UDP data to ESP8266 Node started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_sub_;
    
    int socket_fd_;
    struct sockaddr_in server_addr_;

    void init_udp()
    {
        // 1. Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
            return;
        }

        // 2. Zero out the server address structure
        memset(&server_addr_, 0, sizeof(server_addr_));

        // 3. Fill in the ESP8266's address information
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(ESP8266_PORT);

        // Convert the IP address string to a binary network address
        if (inet_pton(AF_INET, ESP8266_IP, &server_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address / Address not supported: %s", ESP8266_IP);
            close(socket_fd_);
            socket_fd_ = -1; // Mark as invalid
            return;
        }

        RCLCPP_INFO(this->get_logger(), "UDP socket initialized. Sending to %s:%d", ESP8266_IP, ESP8266_PORT);
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Check if the socket was initialized successfully
        if (socket_fd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP socket not initialized. Message ignored.");
            return;
        }

        const std::string& data = msg->data;

        // Use sendto() to send the data as a UDP packet
        ssize_t sent_bytes = sendto(socket_fd_, data.c_str(), data.length(), 0,
                                    (const struct sockaddr *)&server_addr_, sizeof(server_addr_));

        if (sent_bytes < 0) {
            // Error occurred
            RCLCPP_ERROR(this->get_logger(), "Error sending UDP packet: %s", strerror(errno));
        } else if (sent_bytes != (ssize_t)data.length()) {
            // This is unlikely in UDP, but good to check
            RCLCPP_WARN(this->get_logger(), "UDP packet truncated. Sent %ld, expected %zu", sent_bytes, data.length());
        } else {
            // Success
            RCLCPP_INFO(this->get_logger(), "Sent UDP Data: %s", data.c_str());
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Use the new class name
    auto node = std::make_shared<PublishUdpESP8266Node>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
