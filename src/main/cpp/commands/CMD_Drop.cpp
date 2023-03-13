// #include "commands/CMD_Drop.h"
// #include "RobotContainer.h"
// #include "Constants.h"

// CMD_Drop::CMD_Drop(double arm_ext) :
// current_arm_ext(arm_ext){}

// void CMD_Drop::Initialize() {}

// void CMD_Drop::Execute() 
// {
//   auto rc = RobotContainer::get();

//   rc->p_grip_solenoid->Set(true);
//   rc->m_wrist_y.Set(ControlMode::Position, rc->wrist_y_pid.Calculate(rc->wrist_y_degrees, 0.0));
  
//   if (current_arm_ext <= arm_ext_setpoint)
//       rc->m_arm_ext.Set(ControlMode::PercentOutput, 0.2);
//   else if (current_arm_ext >= arm_ext_setpoint)
//       rc->m_arm_ext.Set(ControlMode::PercentOutput, -0.2);
//   else
//       rc->m_arm_ext.Set(ControlMode::PercentOutput, 0.0);
// }

// void CMD_Drop::End(bool interrupted) {}

// bool CMD_Drop::IsFinished() 
// {
//   auto rc = RobotContainer::get();

//   rc->p_grip_solenoid->Set(false);
//   return false;
// }
