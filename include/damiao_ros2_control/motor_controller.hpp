#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "damiao_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace damiao_ros
{

class MotorController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  
  struct MotorConfig {
    std::string name;
    damiao::DM_Motor_Type type;
    uint32_t slave_id;
    uint32_t master_id;
    damiao::Control_Mode control_mode;
  };
  
  explicit MotorController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MotorController();
  
private:
  void init_parameters();
  void load_motor_config();
  void setup_publishers();
  void setup_subscribers();
  void setup_services();
  void setup_action_server();
  
  void control_timer_callback();
  void publish_motor_states();
  
  // Service callbacks
  void handle_enable_motor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response,
    uint32_t motor_id);
    
  void handle_set_zero(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response,
    uint32_t motor_id);
  
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
    
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute_trajectory(const std::shared_ptr<GoalHandle> goal_handle);
  
  // 电机控制模式切换
  void switch_to_position_control();
  void switch_to_velocity_control();
  void switch_to_torque_control();
  
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // 接口
  DamiaoInterface::SharedPtr damiao_interface_;
  
  // 配置
  std::string serial_port_;
  int baud_rate_;
  std::map<uint32_t, MotorConfig> motor_configs_;
  
  // ROS2 接口
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::map<std::string, rclcpp::Publisher<control_msgs::msg::JointControllerState>::SharedPtr> motor_state_pubs_;
  
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  
  // 服务接口
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> enable_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> zero_services_;
};

} // namespace damiao_ros

#endif // MOTOR_CONTROLLER_HPP