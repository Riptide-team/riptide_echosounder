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
    this->declare_parameter("speed_of_sound", 1481.);
    this->declare_parameter("range", 15.); // 1 - 120
    this->declare_parameter("filter_size", 10); // 1 - 32
    this->declare_parameter("threshold", 89); // 0 - 255
    this->declare_parameter("lockout", 0.5); // min 0.5

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
    RCLCPP_DEBUG(rclcpp::get_logger("EchosounderHardware"), "RANGE message witten! %d/%ld char written", count, command.size());

    // TODO MSALT,TRIGGER,0 at the destruction of the node


    raw_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("raw_altitude", 10);
    processed_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("processed_altitude", 10);

    // SeaScanEcho::Reply s_reset(data.substr(0, count));
    // std::vector<std::string> fields = s_reset.Fields();
    // if (!s_reset.Valid() && fields[0] != "MSALT" && fields[1] != "INFO") {
    //     RCLCPP_FATAL(
    //         rclcpp::get_logger("EchosounderHardware"),
    //         "Bad response to RESET: '%s' : got %s, %s", (data.substr(0, count)).c_str(), (std::string(fields[0])).c_str(), (std::string(fields[1])).c_str()
    //     );
    //     return hardware_interface::CallbackReturn::ERROR;
    // }
}

void RiptideEchosounder::configure_echosounder() {
    // COmmand buffer
    std::string command;
    int count;

    // Send RESET message
    SeaScanEcho::Command reset_command = SeaScanEcho::Command({"MSALT", "RESET"});
    command = reset_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "RESET message witten! %d/%ld char written", count, command.size());

    std::this_thread::sleep_for(10ms);

    // Speed of sound
    std::string sos = this->get_parameter("speed_of_sound").as_double();
    SeaScanEcho::Command sos_command = SeaScanEcho::Command({"MSALT", "SOS", std::to_string(sos)});
    command = sos_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "SOS message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(10ms);

    // Filter
    std::string filter = this->get_parameter("filter_size").as_int();
    SeaScanEcho::Command filter_command = SeaScanEcho::Command({"MSALT", "FILTER", std::to_string(filter)});
    command = filter_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "FILTER message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(10ms);

    // LOCKOUT
    std::string lockout = this->get_parameter("lockout").as_double();
    SeaScanEcho::Command lockout_command = SeaScanEcho::Command({"MSALT", "LOCKOUT", std::to_string(lockout)});
    command = lockout_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "LOCKOUT message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(10ms);

    // THRESHOLD
    std::string threshold = this->get_parameter("threshold").as_int();
    SeaScanEcho::Command threshold_command = SeaScanEcho::Command({"MSALT", "THRESHOLD", std::to_string(threshold)});
    command = threshold_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "THRESHOLD message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(10ms);

    // RANGE
    std::string range = this->get_parameter("range").as_double();
    SeaScanEcho::Command range_command = SeaScanEcho::Command({"MSALT", "RANGE", std::to_string(range)});
    command = range_command();
    count = serial_->write(command.size(), (const uint8_t*)command.c_str());
    RCLCPP_DEBUG(this->get_logger(), "RANGE message witten! %d/%ld char written", count, command.size());
    
    std::this_thread::sleep_for(10ms);
}

void RiptideEchosounder::read_callback(const rtac::asio::SerialStream::ErrorCode& err, std::size_t count) {
    // Serial error reading
    if (err) {
        RCLCPP_WARN(this->get_logger(), "Error while serial reading: %s", (err.message()).c_str());
    }

    try {
        SeaScanEcho::Reply s(read_buffer_);
        if (s.Valid()) {
            std::vector<std::string> fields = s.Fields();
            std::size_t i=0;

            if ((fields.size() == 4) and (fields[0] == "MSALT") and (fields[1] == "DATA")) {
                double raw_distance = std::stod(fields[2]);
                double processed_distance = std::stod(fields[2]);

                publish_range(raw_publisher_, raw_distance);
                publish_range(processed_publisher_, processed_distance);
            }
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Invalid frame %s", (read_buffer_).c_str());
        }
    }
    catch (...) {
        RCLCPP_WARN(this->get_logger(), "Error while parsing reply %s", (read_buffer_).c_str());
    }

    // Adding received data to the nmea parser
    // for (std::size_t i = 0; i < count; ++i){
    //     try {
    //         parser.readByte(read_buffer_[i]);
    //     }
    //     catch (nmea::NMEAParseError& e){
    //         RCLCPP_WARN(
    //             this->get_logger(),
    //             "Error while parsing NMEA data (%s)!", (e.what()).c_str()
    //         );
    //     }
    // }

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