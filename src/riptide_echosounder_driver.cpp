#include "riptide_echosounder/riptide_echosounder.hpp"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto echosounder_node = std::make_shared<RiptideEchosounder>();
    rclcpp::spin(echosounder_node);
    rclcpp::on_shutdown(std::bind(&RiptideEchosounder::stop_measurements, echosounder_node.get()));
    rclcpp::shutdown();
    return 0;
}