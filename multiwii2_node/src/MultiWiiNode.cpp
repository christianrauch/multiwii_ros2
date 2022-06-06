#include <MultiWiiNode.hpp>
#include <class_loader/register_macro.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

static const std::set<std::string> sub_params = {
    "imu", "motor", "rc", "attitude", "altitude", "analog", "voltage", "current", "battery"
};

MultiWiiNode::MultiWiiNode() : Node("multiwii"), tf_broadcaster(this)
{
    declare_parameter("device_path", "/dev/ttyUSB0");
    declare_parameter("baud", 115200);

    for(const std::string& sub_param : sub_params) {
        declare_parameter("sub/"+sub_param);
    }

    const std::string device_path = get_parameter("device_path").as_string();
    const size_t baud = get_parameter("baud").as_int();

    std::cout << "open " << device_path << "@" << std::to_string(baud) << std::endl;

    fcu = std::make_unique<fcu::FlightController>();
    fcu->connect(device_path, baud);

    pub_imu_raw = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    pub_imu_mag = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
    pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("local_position/pose", rclcpp::SensorDataQoS());
    pub_rc_in = create_publisher<mavros_msgs::msg::RCIn>("rc/in", rclcpp::SensorDataQoS());
    pub_motors = create_publisher<mavros_msgs::msg::RCOut>("motors", rclcpp::SensorDataQoS());
    pub_battery = create_publisher<sensor_msgs::msg::BatteryState>("battery", rclcpp::SensorDataQoS());
    pub_altitude = create_publisher<std_msgs::msg::Float64>("global_position/rel_alt", rclcpp::SensorDataQoS());
    pub_state = create_publisher<mavros_msgs::msg::State>("state", rclcpp::SensorDataQoS());

    param_cb_hndl = add_on_set_parameters_callback(std::bind(&MultiWiiNode::onParameterChange, this, std::placeholders::_1));

    sub_rc_in = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "rc/override", rclcpp::SystemDefaultsQoS(),
                std::bind(&MultiWiiNode::rc_override_AERT1234, this, std::placeholders::_1));
    sub_rc_in_raw = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "rc/override/raw", rclcpp::SystemDefaultsQoS(),
                std::bind(&MultiWiiNode::rc_override_raw, this, std::placeholders::_1));

    timer_state = create_wall_timer(100ms, std::bind(&MultiWiiNode::onState, this));

    // subscribe with default period
    set_parameter({"sub/imu", 0.01});       // 102
    set_parameter({"sub/motor", 0.1});      // 104
    set_parameter({"sub/rc", 0.1});         // 105
    set_parameter({"sub/attitude", 0.1});   // 108
    set_parameter({"sub/altitude", 0.1});   // 109
    set_parameter({"sub/analog", 0.1});     // 110
    set_parameter({"sub/voltage", 1});      // 128
    set_parameter({"sub/current", 1});      // 129
    set_parameter({"sub/battery", 1});      // 130
}

bool MultiWiiNode::subscribe(const std::string &topic, const double period) {
    if (topic=="imu")
        fcu->subscribe(&MultiWiiNode::onImu, this, period);
    else if (topic=="attitude")
        fcu->subscribe(&MultiWiiNode::onAttitude, this, period);
    else if (topic=="rc")
        fcu->subscribe(&MultiWiiNode::onRc, this, period);
    else if (topic=="motor")
        fcu->subscribe(&MultiWiiNode::onMotor, this, period);
    else if (topic=="analog")
        fcu->subscribe(&MultiWiiNode::onAnalog, this, period);
    else if (topic=="altitude")
        fcu->subscribe(&MultiWiiNode::onAltitude, this, period);
    else if (topic=="voltage")
        fcu->subscribe(&MultiWiiNode::onVoltage, this, period);
    else if (topic=="current")
        fcu->subscribe(&MultiWiiNode::onCurrent, this, period);
    else if (topic=="battery")
        fcu->subscribe(&MultiWiiNode::onBattery, this, period);
    else
        return false;

    return true;
}

rcl_interfaces::msg::SetParametersResult MultiWiiNode::onParameterChange(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
        const std::string name = parameter.get_name();
        static const std::string prefix = "sub/";
        const std::string sub_name = name.substr(prefix.length(), name.length());
        if (name.substr(0, prefix.length()) == prefix &&
            sub_params.count(sub_name) &&
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            if(!subscribe(sub_name, parameter.as_double())) {
                result.successful = false;
                result.reason = "unknown subscriber";
            }
        }
        else {
            result.successful = false;
            result.reason = "unknown or invalid parameter";
        }
    }
    return result;
}

void MultiWiiNode::onImu(const msp::msg::RawImu &imu) {
    const msp::msg::ImuSI imu_si(imu, 512.0, 1/4.096, 0.092, 9.80665);

    std_msgs::msg::Header hdr;
    hdr.stamp = clk.now();
    hdr.frame_id = "multiwii";

    // raw imu data without orientation
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header = hdr;
    imu_msg.linear_acceleration.x = imu_si.acc[0];
    imu_msg.linear_acceleration.y = imu_si.acc[1];
    imu_msg.linear_acceleration.z = imu_si.acc[2];
    imu_msg.angular_velocity.x = deg2rad(imu_si.gyro[0]);
    imu_msg.angular_velocity.y = deg2rad(imu_si.gyro[1]);
    imu_msg.angular_velocity.z = deg2rad(imu_si.gyro[2]);
    pub_imu_raw->publish(imu_msg);

    // magnetic field vector
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header = hdr;
    mag_msg.magnetic_field.x = imu_si.mag[0] * 1e-6;
    mag_msg.magnetic_field.y = imu_si.mag[1] * 1e-6;
    mag_msg.magnetic_field.z = imu_si.mag[2] * 1e-6;
    pub_imu_mag->publish(mag_msg);
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

    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose_stamped.header;
    tf.child_frame_id = "attitude";
    tf.transform.rotation = pose_stamped.pose.orientation;
    tf_broadcaster.sendTransform(tf);

    pub_pose->publish(pose_stamped);
}

void MultiWiiNode::onRc(const msp::msg::Rc &rc) {
    mavros_msgs::msg::RCIn rc_msg;
    rc_msg.header.stamp = clk.now();
    rc_msg.header.frame_id = "multiwii";
    rc_msg.channels = rc.channels;

    pub_rc_in->publish(rc_msg);
}

void MultiWiiNode::onMotor(const msp::msg::Motor &motor) {
    mavros_msgs::msg::RCOut motor_out;
    motor_out.header.stamp = clk.now();
    motor_out.header.frame_id = "multiwii";
    for(const uint16_t m : motor.motor) {
        motor_out.channels.push_back(m);
    }
    pub_motors->publish(motor_out);
}

void MultiWiiNode::onAnalog(const msp::msg::Analog &analog) {
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = clk.now();
    battery.header.frame_id = "multiwii";
    battery.voltage = analog.vbat;
    battery.current = analog.amperage;

    pub_battery->publish(battery);
}

void MultiWiiNode::onAltitude(const msp::msg::Altitude &altitude) {
    std_msgs::msg::Float64 alt; // altitude in meter
    alt.data = altitude.altitude;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = clk.now();
    tf.header.frame_id = "multiwii";
    tf.child_frame_id = "altitude";
    tf.transform.translation.z = altitude.altitude;
    tf_broadcaster.sendTransform(tf);

    pub_altitude->publish(alt);
}

void MultiWiiNode::onVoltage(const msp::msg::VoltageMeters &voltage_meters) {
//    std::cout << voltage_meters << std::endl;
    (void)voltage_meters;
}

void MultiWiiNode::onCurrent(const msp::msg::CurrentMeters &current_meters) {
//    std::cout << current_meters << std::endl;
    (void)current_meters;
}

void MultiWiiNode::onBattery(const msp::msg::BatteryState &battery_state) {
//    std::cout << battery_state << std::endl;
    (void)battery_state;
}

void MultiWiiNode::onState() {
    mavros_msgs::msg::State state;
    state.armed = fcu->isArmed();
    pub_state->publish(state);
}

void MultiWiiNode::rc_override_AERT1234(const mavros_msgs::msg::OverrideRCIn::SharedPtr rc) {
    // set channels in order of
    // AERT (Aileron, Elevator, Rudder, Throttle), or
    // RPYT (Roll, Pitch, Yaw, Throttle) respectively
    fcu->setRc(rc->channels[0], rc->channels[1], rc->channels[2], rc->channels[3],
               rc->channels[4], rc->channels[5], rc->channels[6], rc->channels[7]);
}

void MultiWiiNode::rc_override_raw(const mavros_msgs::msg::OverrideRCIn::SharedPtr rc) {
    std::vector<uint16_t> channels;
    for(const uint16_t c : rc->channels) { channels.push_back(c); }
    fcu->setRc(channels);
}

CLASS_LOADER_REGISTER_CLASS(MultiWiiNode, rclcpp::Node)
