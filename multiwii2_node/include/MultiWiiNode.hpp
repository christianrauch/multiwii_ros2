#ifndef MULTIWIINODE_HPP
#define MULTIWIINODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <multiwii_msgs/srv/receive_msp_raw_message.hpp>
#include <multiwii_msgs/srv/send_msp_raw_message.hpp>

#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
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

private:
    std::unique_ptr<fcu::FlightController> fcu;

    rclcpp::Clock clk;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr pub_rc_in;
    rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr pub_motors;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_altitude;

    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_rc_in;
    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_rc_in_raw;

    static double deg2rad(const double deg) {
        return deg/180.0 * M_PI;
    }

    static double rad2deg(const double rad) {
        return rad/M_PI * 180.0;
    }

    void onImu(const msp::msg::RawImu &imu);

    void onAttitude(const msp::msg::Attitude &attitude);

    void onRc(const msp::msg::Rc &rc);

    void onMotor(const msp::msg::Motor &motor);

    void onAnalog(const msp::msg::Analog &analog);

    void onAltitude(const msp::msg::Altitude &altitude);

    void rc_override_AERT1234(const mavros_msgs::msg::OverrideRCIn::SharedPtr rc) {
        fcu->setRc(rc->channels[0], rc->channels[1], rc->channels[2], rc->channels[3],
                   rc->channels[4], rc->channels[5], rc->channels[6], rc->channels[7]);
    }

    void rc_override_raw(const mavros_msgs::msg::OverrideRCIn::SharedPtr rc) {
        std::vector<uint16_t> channels;
        for(const uint16_t c : rc->channels) { channels.push_back(c); }
        fcu->setRc(channels);
    }
};

#endif // MULTIWIINODE_HPP
