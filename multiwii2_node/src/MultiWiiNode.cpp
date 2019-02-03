#include <MultiWiiNode.hpp>
#include <class_loader/register_macro.hpp>

MultiWiiNode::MultiWiiNode() : Node("multiwii") {
    fcu = std::make_unique<fcu::FlightController>();
    fcu->connect("/dev/ttyUSB0", 500000);

    pub_imu = create_publisher<sensor_msgs::msg::Imu>("imu/data");
    pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("local_position/pose");
    pub_rc_in = create_publisher<mavros_msgs::msg::RCIn>("rc/in");
    pub_motors = create_publisher<mavros_msgs::msg::RCOut>("motors");
    pub_battery = create_publisher<sensor_msgs::msg::BatteryState>("battery");
    pub_altitude = create_publisher<std_msgs::msg::Float64>("global_position/rel_alt");

    sub_rc_in = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "rc/override", std::bind(&MultiWiiNode::rc_override_AERT1234, this, std::placeholders::_1), qos);
    sub_rc_in_raw = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "rc/override/raw", std::bind(&MultiWiiNode::rc_override_raw, this, std::placeholders::_1), qos);

    fcu->subscribe(&MultiWiiNode::onImu, this, 0.01);
    fcu->subscribe(&MultiWiiNode::onAttitude, this, 0.1);
    fcu->subscribe(&MultiWiiNode::onRc, this, 0.1);
    fcu->subscribe(&MultiWiiNode::onMotor, this, 0.1);
    fcu->subscribe(&MultiWiiNode::onAnalog, this, 0.1);
    fcu->subscribe(&MultiWiiNode::onAltitude, this, 0.1);
}

void MultiWiiNode::onImu(const msp::msg::RawImu &imu) {
    const msp::msg::ImuSI imu_si(imu, 512.0, 1/4.096, 0.092, 9.80665);

    sensor_msgs::msg::Imu imu_msg;

    imu_msg.header.stamp = clk.now();
    imu_msg.header.frame_id = "multiwii";

    imu_msg.linear_acceleration.x = imu_si.acc[0];
    imu_msg.linear_acceleration.y = imu_si.acc[1];
    imu_msg.linear_acceleration.z = imu_si.acc[2];

    imu_msg.angular_velocity.x = deg2rad(imu_si.gyro[0]);
    imu_msg.angular_velocity.y = deg2rad(imu_si.gyro[1]);
    imu_msg.angular_velocity.z = deg2rad(imu_si.gyro[2]);

    // rotation from direction of acceleration and magnetic field
    const Eigen::Vector3d magn(imu.mag[0], imu.mag[1], imu.mag[2]);
    const Eigen::Vector3d lin_acc(imu.acc[0], imu.acc[1], imu.acc[2]);

    // http://www.camelsoftware.com/2016/02/20/imu-maths/
    Eigen::Matrix3d rot;
    rot.col(0) = lin_acc.cross(magn).cross(lin_acc).normalized();
    rot.col(1) = lin_acc.cross(magn).normalized();
    rot.col(2) = lin_acc.normalized();

    const Eigen::Quaterniond orientation(rot);
    imu_msg.orientation.x = orientation.x();
    imu_msg.orientation.y = orientation.y();
    imu_msg.orientation.z = orientation.z();
    imu_msg.orientation.w = orientation.w();

    pub_imu->publish(imu_msg);
}

void MultiWiiNode::onAttitude(const msp::msg::Attitude &attitude) {
    // r,p,y to rotation matrix
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(deg2rad(attitude.roll), Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(deg2rad(attitude.pitch),  Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(deg2rad(attitude.yaw), Eigen::Vector3f::UnitZ());

    const Eigen::Quaternionf quat(rot);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = clk.now();
    pose_stamped.header.frame_id = "multiwii";
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();

    pub_pose->publish(pose_stamped);
}

void MultiWiiNode::onRc(const msp::msg::Rc &rc) {
    mavros_msgs::msg::RCIn rc_msg;
    rc_msg.header.stamp = clk.now();
    rc_msg.channels = rc.channels;

    pub_rc_in->publish(rc_msg);
}

void MultiWiiNode::onMotor(const msp::msg::Motor &motor) {
    mavros_msgs::msg::RCOut motor_out;
    for(const uint16_t m : motor.motor) {
        motor_out.channels.push_back(m);
    }
    pub_motors->publish(motor_out);
}

void MultiWiiNode::onAnalog(const msp::msg::Analog &analog) {
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = clk.now();
    battery.voltage = analog.vbat;
    battery.current = analog.amperage;

    pub_battery->publish(battery);
}

void MultiWiiNode::onAltitude(const msp::msg::Altitude &altitude) {
    std_msgs::msg::Float64 alt; // altitude in meter
    alt.data = altitude.altitude;
    pub_altitude->publish(alt);
}

const rmw_qos_profile_t MultiWiiNode::qos = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    false};

CLASS_LOADER_REGISTER_CLASS(MultiWiiNode, rclcpp::Node)
