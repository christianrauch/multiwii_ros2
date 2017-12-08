#include <MultiWiiNode.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiWiiNode>());
    rclcpp::shutdown();
    return 0;
}
