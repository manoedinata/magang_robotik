// Receive sensor data from ESP8266 via Wi-Fi UDP
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <string>

// C/C++ Networking Libraries
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring> // For memset
#include <cerrno>  // For errno

// --- UDP Configuration ---
// This is the port this ROS node will listen on
#define ROS_LISTEN_PORT 4211

class ReceiveSensorsUdpNode : public rclcpp::Node
{
public:
    ReceiveSensorsUdpNode() : Node("receive_sensors_udp_node"), socket_fd_(-1) {}

    // Destructor to close the socket
    ~ReceiveSensorsUdpNode()
    {
        if (socket_fd_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Closing UDP socket.");
            close(socket_fd_);
        }
    }

    void init()
    {
        sensors_pub_ = this->create_publisher<std_msgs::msg::String>("sensors", 10);

        // Start the UDP listening loop in a separate thread
        std::thread([this]() { this->udp_listen_loop(); }).detach();
        
        RCLCPP_INFO(this->get_logger(), "UDP Sensor Receiver Node started.");
    }
    
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensors_pub_;
    int socket_fd_;

    void udp_listen_loop()
    {
        // 1. Create UDP socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
            return;
        }

        // 2. Prepare the server address structure
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all local IPs
        server_addr.sin_port = htons(ROS_LISTEN_PORT);

        // 3. Bind the socket to the port
        if (bind(socket_fd_, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to port %d: %s", ROS_LISTEN_PORT, strerror(errno));
            close(socket_fd_);
            socket_fd_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "UDP listener bound to port %d. Waiting for data...", ROS_LISTEN_PORT);

        char buffer[1024];
        struct sockaddr_in client_addr; // To store sender's info
        socklen_t client_len = sizeof(client_addr);

        // 4. Main listening loop
        while (rclcpp::ok())
        {
            // This call is blocking
            ssize_t len = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, 0,
                                   (struct sockaddr *)&client_addr, &client_len);

            if (!rclcpp::ok()) {
                // Node is shutting down
                break;
            }

            if (len < 0) {
                RCLCPP_ERROR(this->get_logger(), "recvfrom() failed: %s", strerror(errno));
                continue;
            }

            // Null-terminate the received data
            buffer[len] = '\0';

            // Publish the received data as a ROS message
            auto msg = std_msgs::msg::String();
            msg.data = std::string(buffer);
            sensors_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Received UDP and published to /sensors: '%s'", msg.data.c_str());
        }

        // Clean up socket
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        RCLCPP_INFO(this->get_logger(), "UDP listener thread shutting down.");
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReceiveSensorsUdpNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
