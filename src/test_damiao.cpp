#include "rclcpp/rclcpp.hpp"
#include "damiao_ros2_control/damiao.h"
#include "damiao_ros2_control/SerialPort.h"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <memory>
#include <cmath>
#include <vector>
#include <string>

class GimbalControlNode : public rclcpp::Node
{
public:
    GimbalControlNode()
    : Node("gimbal_control_node")
    {
        // ----��̬��������----
        this->declare_parameter("trajectory_mode", 2);  // 0: sin, 1: circle, 2: rect
        this->declare_parameter("amplitude_x", 5.0);
        this->declare_parameter("amplitude_y", 5.0);
        this->declare_parameter("frequency", 1.0);
        this->declare_parameter("wall_length", 20.0);
        this->declare_parameter("wall_width", 15.0);
        this->declare_parameter("D", 80.0);
        this->declare_parameter("scan_speed", 1.0);
        // ����
        this->declare_parameter("max_delta", 1.0);
        this->declare_parameter("alpha", 1.0);

        print_all_parameters();

        serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        dm_ = damiao::Motor_Control(serial_);

        M5_ = std::make_shared<damiao::Motor>(damiao::DMG6220, 0x05, 0x15);
        M6_ = std::make_shared<damiao::Motor>(damiao::DMH3510, 0x06, 0x16);
        dm_.addMotor(M5_.get());
        dm_.addMotor(M6_.get());

        if (dm_.switchControlMode(*M5_, damiao::POS_VEL_MODE)) {
            RCLCPP_INFO(this->get_logger(), "M5 switched to POS_VEL_MODE successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch M5 mode!");
        }
        if (dm_.switchControlMode(*M6_, damiao::POS_VEL_MODE)) {
            RCLCPP_INFO(this->get_logger(), "M6 switched to POS_VEL_MODE successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch M6 mode!");
        }

        dm_.save_motor_param(*M5_);
        dm_.save_motor_param(*M6_);
        dm_.set_zero_position(*M5_);
        dm_.set_zero_position(*M6_);
        dm_.enable(*M5_);
        dm_.enable(*M6_);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&GimbalControlNode::on_parameter_change, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/gimbal/laser_marker", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&GimbalControlNode::control_gimbal, this)
        );

        RCLCPP_INFO(this->get_logger(), "Gimbal control node initialized!");
    }

    ~GimbalControlNode() override
    {
        dm_.disable(*M5_);
        dm_.disable(*M6_);
    }

private:
    // ----��Ա����----
    std::shared_ptr<damiao::Motor> M5_;
    std::shared_ptr<damiao::Motor> M6_;
    std::shared_ptr<SerialPort> serial_;
    damiao::Motor_Control dm_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::vector<geometry_msgs::msg::Point> traj_points_;
    double t_ = 0.0;

    // ��������һ���˲����Ŀ��
    double prev_x_ = 0.0;
    double prev_y_ = 0.0;

    void print_all_parameters() {
        RCLCPP_INFO(this->get_logger(), "Parameters after declaration:");
        RCLCPP_INFO(this->get_logger(), "  trajectory_mode = %ld", this->get_parameter("trajectory_mode").as_int());
        RCLCPP_INFO(this->get_logger(), "  amplitude_x = %.4f", this->get_parameter("amplitude_x").as_double());
        RCLCPP_INFO(this->get_logger(), "  amplitude_y = %.4f", this->get_parameter("amplitude_y").as_double());
        RCLCPP_INFO(this->get_logger(), "  frequency = %.4f", this->get_parameter("frequency").as_double());
        RCLCPP_INFO(this->get_logger(), "  wall_length = %.4f", this->get_parameter("wall_length").as_double());
        RCLCPP_INFO(this->get_logger(), "  wall_width = %.4f", this->get_parameter("wall_width").as_double());
        RCLCPP_INFO(this->get_logger(), "  D = %.4f", this->get_parameter("D").as_double());
        RCLCPP_INFO(this->get_logger(), "  scan_speed = %.4f", this->get_parameter("scan_speed").as_double());
        RCLCPP_INFO(this->get_logger(), "  max_delta = %.4f", this->get_parameter("max_delta").as_double());
        RCLCPP_INFO(this->get_logger(), "  alpha = %.4f", this->get_parameter("alpha").as_double());
    }

rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    for (const auto &param : params) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            RCLCPP_INFO(this->get_logger(), "Parameter %s changed to %.4f", param.get_name().c_str(), param.as_double());
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            RCLCPP_INFO(this->get_logger(), "Parameter %s changed to %ld", param.get_name().c_str(), param.as_int());
        } else {
            RCLCPP_INFO(this->get_logger(), "Parameter %s changed (type not handled)", param.get_name().c_str());
        }
    }
    print_all_parameters();
    return result;
}


    void control_gimbal()
    {
        t_ += 0.03; // ÿ30ms�ۼ�

        int mode         = this->get_parameter("trajectory_mode").as_int();
        double amplitude_x = this->get_parameter("amplitude_x").as_double();
        double amplitude_y = this->get_parameter("amplitude_y").as_double();
        double frequency   = this->get_parameter("frequency").as_double();
        double wall_length = this->get_parameter("wall_length").as_double();
        double wall_width  = this->get_parameter("wall_width").as_double();
        double D           = this->get_parameter("D").as_double();
        double scan_speed  = this->get_parameter("scan_speed").as_double();
        // ����
        double max_delta   = this->get_parameter("max_delta").as_double();
        double alpha       = this->get_parameter("alpha").as_double();

        if (std::abs(amplitude_x) > wall_length / 2.0) amplitude_x = wall_length / 2.0;
        if (std::abs(amplitude_y) > wall_width  / 2.0) amplitude_y = wall_width  / 2.0;

        double raw_x = 0, raw_y = 0;

        // ---------ԭʼĿ��켣����---------
        if (mode == 0) { // ���Ҳ�
            raw_x = -wall_length/2.0 + fmod(scan_speed * t_, wall_length);
            raw_y = amplitude_y * std::sin(2 * M_PI * frequency * raw_x / wall_length);
        } else if (mode == 1) { // Բ
            double theta = 2 * M_PI * frequency * t_;
            raw_x = amplitude_x * std::cos(theta);
            raw_y = amplitude_y * std::sin(theta);
        } else if (mode == 2) { // ����
double T = wall_length / scan_speed;
double phase = fmod(t_, T) / T;
double half_x = amplitude_x;
double half_y = amplitude_y;

// ˳ʱ���Ķι켣
if (phase < 0.25) { // ����
    raw_x = -half_x + 2 * half_x * (phase / 0.25);
    raw_y = -half_y;
} else if (phase < 0.5) { // �µ���
    raw_x = half_x;
    raw_y = -half_y + 2 * half_y * ((phase - 0.25) / 0.25);
} else if (phase < 0.75) { // �ҵ���
    raw_x = half_x - 2 * half_x * ((phase - 0.5) / 0.25);
    raw_y = half_y;
} else { // �ϵ���
    raw_x = -half_x;
    raw_y = half_y - 2 * half_y * ((phase - 0.75) / 0.25);
}

        }

        // �߽紦��
        if (raw_x >  wall_length/2.0)  raw_x -= wall_length;
        if (raw_x < -wall_length/2.0)  raw_x += wall_length;
        if (raw_y >  wall_width/2.0)   raw_y -= wall_width;
        if (raw_y < -wall_width/2.0)   raw_y += wall_width;

        // --------- Ŀ���޷� + һ���˲� ---------
        double delta_x = raw_x - prev_x_;
        double delta_y = raw_y - prev_y_;
        if (delta_x >  max_delta) delta_x =  max_delta;
        if (delta_x < -max_delta) delta_x = -max_delta;
        if (delta_y >  max_delta) delta_y =  max_delta;
        if (delta_y < -max_delta) delta_y = -max_delta;

        double limit_x = prev_x_ + delta_x;
        double limit_y = prev_y_ + delta_y;

        double filt_x = alpha * limit_x + (1 - alpha) * prev_x_;
        double filt_y = alpha * limit_y + (1 - alpha) * prev_y_;

        prev_x_ = filt_x;
        prev_y_ = filt_y;

        // ������̨
        double theta_x = std::atan2(filt_x, D);
        double theta_y = std::atan2(filt_y, D);
        dm_.control_pos_vel(*M5_, theta_x, 5.0);
        dm_.control_pos_vel(*M6_, theta_y, 5.0);

        // �洢�������켣��
        geometry_msgs::msg::Point pt;
        pt.x = filt_x;
        pt.y = filt_y;
        pt.z = D;
        traj_points_.push_back(pt);
        if (traj_points_.size() > 2000) traj_points_.erase(traj_points_.begin());

        // Marker: �켣
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "laser_traj";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.01;
        line_strip.color.a = 1.0;
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.3;
        line_strip.color.b = 0.0;
        line_strip.points = traj_points_;
        marker_pub_->publish(line_strip);

        // Marker: ��ǰ��
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = this->now();
        sphere.ns = "laser_traj";
        sphere.id = 1;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position = pt;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.05;
        sphere.scale.y = 0.05;
        sphere.scale.z = 0.05;
        sphere.color.a = 1.0;
        sphere.color.g = 1.0;
        marker_pub_->publish(sphere);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
