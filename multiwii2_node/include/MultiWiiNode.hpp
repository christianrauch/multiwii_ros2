#ifndef MULTIWIINODE_HPP
#define MULTIWIINODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <multiwii_msgs/srv/receive_msp_raw_message.hpp>
#include <multiwii_msgs/srv/send_msp_raw_message.hpp>

#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <eigen3/Eigen/Geometry>

#include <msp/FlightController.hpp>

class MultiWiiNode : public rclcpp::Node {
public:
    MultiWiiNode();

    void onImu(const msp::msg::ImuRaw &imu);

private:
    std::unique_ptr<fcu::FlightController> fcu;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
};

#endif // MULTIWIINODE_HPP
