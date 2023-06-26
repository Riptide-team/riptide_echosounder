#include "riptide_echosounder/riptide_echosounder.hpp"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RiptideEchosounder>());
    rclcpp::shutdown();
    return 0;
}