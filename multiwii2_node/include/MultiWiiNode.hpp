#ifndef MULTIWIINODE_HPP
#define MULTIWIINODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
#include <mavros_msgs/msg/state.hpp>

#include <eigen3/Eigen/Geometry>

#include <msp/FlightController.hpp>

class MultiWiiNode : public rclcpp::Node {
public:
    MultiWiiNode();

private:
    std::unique_ptr<fcu::FlightController> fcu;

    rclcpp::Clock clk;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_imu_mag;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr pub_rc_in;
    rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr pub_motors;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_altitude;
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr pub_state;

    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_rc_in;
    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_rc_in_raw;

    OnSetParametersCallbackHandle::SharedPtr param_cb_hndl;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    rclcpp::TimerBase::SharedPtr timer_state;

    static double deg2rad(const double deg) {
        return deg/180.0 * M_PI;
    }

    static double rad2deg(const double rad) {
        return rad/M_PI * 180.0;
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);

    bool subscribe(const std::string &topic, const double period);

    void onImu(const msp::msg::RawImu &imu);

    void onAttitude(const msp::msg::Attitude &attitude);

    void onRc(const msp::msg::Rc &rc);

    void onMotor(const msp::msg::Motor &motor);

    void onAnalog(const msp::msg::Analog &analog);

    void onAltitude(const msp::msg::Altitude &altitude);

    void onVoltage(const msp::msg::VoltageMeters &voltage_meters);

    void onCurrent(const msp::msg::CurrentMeters &current_meters);

    void onBattery(const msp::msg::BatteryState &battery_state);

    void onState();

    void rc_override_AERT1234(const mavros_msgs::msg::OverrideRCIn::SharedPtr rc);

    void rc_override_raw(const mavros_msgs::msg::OverrideRCIn::SharedPtr rc);
};

#endif // MULTIWIINODE_HPP
