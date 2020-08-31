#include <MultiWiiNode.hpp>
#include <class_loader/register_macro.hpp>

static const std::set<std::string> sub_params = {
    "imu", "motor", "rc", "attitude", "altitude", "analog", "voltage", "current", "battery"
};

MultiWiiNode::MultiWiiNode() : Node("multiwii") {
    declare_parameter("device_path", "/dev/ttyUSB0");
    declare_parameter("baud", 125200);

    for(const std::string& sub_param : sub_params) {
        declare_parameter("sub/"+sub_param);
    }

    const std::string device_path = get_parameter("device_path").as_string();
    const size_t baud = get_parameter("baud").as_int();

    std::cout << "open " << device_path << "@" << std::to_string(baud) << std::endl;

    fcu = std::make_unique<fcu::FlightController>();
    fcu->connect(device_path, baud);

    pub_imu = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("local_position/pose", rclcpp::SensorDataQoS());
    pub_rc_in = create_publisher<mavros_msgs::msg::RCIn>("rc/in", rclcpp::SensorDataQoS());
    pub_motors = create_publisher<mavros_msgs::msg::RCOut>("motors", rclcpp::SensorDataQoS());
    pub_battery = create_publisher<sensor_msgs::msg::BatteryState>("battery", rclcpp::SensorDataQoS());
    pub_altitude = create_publisher<std_msgs::msg::Float64>("global_position/rel_alt", rclcpp::SensorDataQoS());

    param_cb_hndl = add_on_set_parameters_callback(std::bind(&MultiWiiNode::onParameterChange, this, std::placeholders::_1));

    sub_rc_in = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "rc/override", rclcpp::SystemDefaultsQoS(), std::bind(&MultiWiiNode::rc_override_AERT1234, this, std::placeholders::_1));
    sub_rc_in_raw = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
                "rc/override/raw", rclcpp::SystemDefaultsQoS(), std::bind(&MultiWiiNode::rc_override_raw, this, std::placeholders::_1));

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

CLASS_LOADER_REGISTER_CLASS(MultiWiiNode, rclcpp::Node)
