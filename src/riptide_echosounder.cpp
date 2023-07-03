#include "riptide_echosounder/riptide_echosounder.hpp"

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>
#include <SeaScanEcho/Command.hpp>
#include <SeaScanEcho/Reply.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <string>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;


RiptideEchosounder::RiptideEchosounder() : Node("riptide_echosounder") {
    // Parameters
    this->declare_parameter("port", "/dev/Echosounder");
    this->declare_parameter("baud_rate", 38400);

    // Echosounder parameters
    this->declare_parameter("speed_of_sound", "1481");
    this->declare_parameter("range", "15"); // 1 - 120
    this->declare_parameter("filter_size", "10"); // 1 - 32
    this->declare_parameter("threshold", "89"); // 0 - 255
    this->declare_parameter("lockout", "0.5"); // min 0.5

    // Getting serial parameters
    port_ = this->get_parameter("port").as_string();
    baud_rate_ = static_cast<unsigned int>(this->get_parameter("baud_rate").as_int());

    // Creating driver
    try {
        serial_ = rtac::asio::Stream::CreateSerial(port_, baud_rate_);
        serial_->start();
        serial_->flush();
    }
    catch(boost::system::system_error& e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("EchosounderHardware"),
            "Serial error: '%s'", e.what()
        );
    }

    // Launching async read on serial port
    read_buffer_ = std::string(1024, '\0');
    if  (!serial_->async_read_until(
            read_buffer_.size(), reinterpret_cast<std::uint8_t*>(const_cast<char*>(read_buffer_.c_str())), '\n',
            std::bind(&RiptideEchosounder::read_callback, this, std::placeholders::_1, std::placeholders::_2))
        ) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Unable to start async read on serial port `%s` with the baudrate `%d`!", port_.c_str(), baud_rate_
        );
    }

    // Configure echosounder
    configure_echosounder();


    // Launch continuous measurements
    SeaScanEcho::Command trigger_command = SeaScanEcho::Command({"MSALT", "TRIGGER", "2"});
    std::string command = trigger_command();
    int count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "TRIGGER message witten! %d/%ld char written", count, command.size());

    // TODO MSALT,TRIGGER,0 at the destruction of the node

    raw_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("raw_altitude", 10);
    processed_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("processed_altitude", 10);
}

void RiptideEchosounder::configure_echosounder() {
    // COmmand buffer
    std::string command;
    int count;

    // Send RESET message
    SeaScanEcho::Command info_command = SeaScanEcho::Command({"MSALT", "INFO"});
    command = info_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "INFO message witten! %d/%ld char written", count, command.size());

    std::this_thread::sleep_for(100ms);

    // Speed of sound
    std::string sos = this->get_parameter("speed_of_sound").as_string();
    SeaScanEcho::Command sos_command = SeaScanEcho::Command({"MSALT", "SOS", sos});
    command = sos_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "SOS message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(100ms);

    // Filter
    std::string filter = this->get_parameter("filter_size").as_string();
    SeaScanEcho::Command filter_command = SeaScanEcho::Command({"MSALT", "FILTER", filter});
    command = filter_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "FILTER message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(100ms);

    // LOCKOUT
    std::string lockout = this->get_parameter("lockout").as_string();
    SeaScanEcho::Command lockout_command = SeaScanEcho::Command({"MSALT", "LOCKOUT", lockout});
    command = lockout_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "LOCKOUT message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(100ms);

    // RANGE
    std::string range = this->get_parameter("range").as_string();
    SeaScanEcho::Command range_command = SeaScanEcho::Command({"MSALT", "RANGE", range});
    command = range_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "RANGE message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(100ms);

    // THRESHOLD
    std::string threshold = this->get_parameter("threshold").as_string();
    SeaScanEcho::Command threshold_command = SeaScanEcho::Command({"MSALT", "THRESHOLD", threshold});
    command = threshold_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "THRESHOLD message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(100ms);
}

void RiptideEchosounder::read_callback(const rtac::asio::SerialStream::ErrorCode& err, std::size_t count) {
    // Serial error reading
    if (err) {
        RCLCPP_WARN(this->get_logger(), "Error while serial reading: %s", (err.message()).c_str());
    }

    std::string data = read_buffer_.substr(0, count);

    std::cout << "Read Buffer: " << read_buffer_ << std::endl;

    try {
        RCLCPP_DEBUG(this->get_logger(), "Read %ld chars: %s", count, (read_buffer_.substr(0, count)).c_str());
        SeaScanEcho::Reply s(read_buffer_);
        if (s.Valid()) {
            std::vector<std::string> fields = s.Fields();
            if ((fields.size() == 4) and (fields[0] == "MSALT") and (fields[1] == "DATA")) {
                double raw_distance = std::stod(fields[2]);
                double processed_distance = std::stod(fields[3]);

                publish_range(raw_publisher_, raw_distance);
                publish_range(processed_publisher_, processed_distance);
                RCLCPP_DEBUG(this->get_logger(), "Gathered distances: %f, %f", raw_distance, processed_distance);
            }
            RCLCPP_DEBUG(this->get_logger(), "Valid NMEA sentence");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid frame %s", data.c_str());
        }
    }
    catch (...) {
        RCLCPP_WARN(this->get_logger(), "Error while parsing reply %s", data.c_str());
    }

    read_buffer_ = std::string(1024, '\0');
    // Relaunching an async read
    if(!serial_->async_read_until(read_buffer_.size(),
        reinterpret_cast<std::uint8_t*>(const_cast<char*>(read_buffer_.c_str())),
        '\n', std::bind(&RiptideEchosounder::read_callback, this, std::placeholders::_1, std::placeholders::_2)
        )) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Async read until could not start !"
        );
    }
}

void RiptideEchosounder::publish_range(rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher, double distance) {
    sensor_msgs::msg::Range range;

    // Header
    range.header.stamp = this->get_clock()->now();
    range.header.frame_id = "riptide_echosounder";

    // Filling the message
    range.field_of_view = 0.2;
    range.min_range = this->get_parameter("lockout").as_double();
    range.max_range = this->get_parameter("range").as_double();
    range.range = distance;

    publisher->publish(range);
}

RiptideEchosounder::~RiptideEchosounder() {
    RCLCPP_DEBUG(
        this->get_logger(),
        "TRIGGERING measurements stop!"
    );

    // Launch continuous measurements
    SeaScanEcho::Command trigger_command = SeaScanEcho::Command({"MSALT", "TRIGGER", "0"});
    std::string command = trigger_command();
    int count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "TRIGGER message witten! %d/%ld char written", count, command.size());
}