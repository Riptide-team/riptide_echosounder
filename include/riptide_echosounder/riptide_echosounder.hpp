#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>

#include <string>


class RiptideEchosounder : public rclcpp::Node {
  public:
    RiptideEchosounder();

    void stop_measurements();

    private:

        // Configure echosounder
        void configure_echosounder();

        // Serial read callback
        void read_callback(const rtac::asio::SerialStream::ErrorCode& err, std::size_t count);

        // Serial port
        std::string port_;

        // Serial baud rate
        unsigned int baud_rate_;

        // Driver
        rtac::asio::Stream::Ptr serial_ = nullptr;

        // Read buffer
        std::string read_buffer_;

        // Publisher
        void publish_range(rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher, double distance);

        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr raw_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr processed_publisher_;
};

