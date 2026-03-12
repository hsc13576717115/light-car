#ifndef DAMIAO_INTERFACE_HPP
#define DAMIAO_INTERFACE_HPP

#include "damiao.h"
#include <memory>
#include <string>
#include <unordered_map>

namespace damiao_ros
{

class DamiaoInterface
{
public:
  using SharedPtr = std::shared_ptr<DamiaoInterface>;
  
  DamiaoInterface(const std::string & port, speed_t baudrate);
  ~DamiaoInterface();
  
  void add_motor(damiao::Motor * motor);
  void enable_motor(uint32_t motor_id);
  void disable_motor(uint32_t motor_id);
  void set_zero_position(uint32_t motor_id);
  
  void control_mit(uint32_t motor_id, float kp, float kd, float q, float dq, float tau);
  void control_pos_vel(uint32_t motor_id, float pos, float vel);
  void control_vel(uint32_t motor_id, float vel);
  void control_pos_force(uint32_t motor_id, float pos, uint16_t vel, uint16_t i);
  
  void refresh_motor_status(uint32_t motor_id);
  
  float get_position(uint32_t motor_id) const;
  float get_velocity(uint32_t motor_id) const;
  float get_torque(uint32_t motor_id) const;
  
  bool switch_control_mode(uint32_t motor_id, damiao::Control_Mode mode);
  float read_motor_param(uint32_t motor_id, damiao::DM_REG reg);
  bool change_motor_param(uint32_t motor_id, damiao::DM_REG reg, float value);
  void save_motor_param(uint32_t motor_id);

private:
  SerialPort::SharedPtr serial_;
  std::unique_ptr<damiao::Motor_Control> motor_control_;
  std::unordered_map<uint32_t, damiao::Motor*> motors_;
};

} // namespace damiao_ros

#endif // DAMIAO_INTERFACE_HPP