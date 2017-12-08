#include <MultiWiiNode.hpp>
#include <msp/msg_print.hpp>
#include <class_loader/class_loader_register_macro.h>

MultiWiiNode::MultiWiiNode() : Node("multiwii") {
    std::cout << "constructing..." << std::endl;
    fcu = std::make_unique<fcu::FlightController>("/dev/ttyUSB0", 500000);

    std::cout << "initialising..." << std::endl;
    fcu->initialise();

    pub_imu = create_publisher<sensor_msgs::msg::Imu>("imu/data");

//    fcu->subscribe<msp::msg::ImuRaw>([this](const msp::msg::ImuRaw &imu){}, this);
    std::cout << "subscribing..." << std::endl;
    fcu->subscribe(&MultiWiiNode::onImu, this, 0.1);
}

void MultiWiiNode::onImu(const msp::msg::ImuRaw &imu) {
    const msp::msg::ImuSI imu_si(imu, 512.0, 1/4.096, 0.092, 9.80665);
    std::cout << imu_si;
}

CLASS_LOADER_REGISTER_CLASS(MultiWiiNode, rclcpp::Node)
