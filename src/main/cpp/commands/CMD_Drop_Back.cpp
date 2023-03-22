// // #include "commands/CMD_Drop_Back.h"
// // #include "RobotContainer.h"
// // #include "Constants.h"

// // CMD_Drop::CMD_Drop(double arm_ext) :
// // current_arm_ext(arm_ext){}

// // void CMD_Drop::Initialize() {}

// // void CMD_Drop::Execute() 
// // {
// //   auto rc = RobotContainer::get();

// //   rc->p_grip_solenoid->Set(true);
// //   rc->m_wrist_y.Set(ControlMode::Position, rc->wrist_y_pid.Calculate(rc->wrist_y_degrees, 180.0));
  
// //   if (current_arm_ext <= arm_ext_setpoint)
// //       rc->m_arm_ext.Set(ControlMode::PercentOutput, 0.2);
// //   else if (current_arm_ext >= arm_ext_setpoint)
// //       rc->m_arm_ext.Set(ControlMode::PercentOutput, -0.2);
// //   else
// //       rc->m_arm_ext.Set(ControlMode::PercentOutput, 0.0);
// // }

// // void CMD_Drop::End(bool interrupted) {}

// // bool CMD_Drop::IsFinished() 
// // {
// //   auto rc = RobotContainer::get();

// //   rc->p_grip_solenoid->Set(false);
// //   return false;
// // }
////////////////

 #include "commands/CMD_Drop_Back.h"
 #include "RobotContainer.h"
 #include "Constants.h"

 t34::CMD_Drop_Back::CMD_Drop_Back(std::string placement) :
    placement(placement)
    , setpoint(96.0){}

 void t34::CMD_Drop_Back::Initialize() {
    auto rc = RobotContainer::get();

    if (placement == "TOP CUBE") {
        setpoint = 96.0;
    }

    else if (placement == "MID CUBE") {
        setpoint = 86.0;
    }

    rc->arm_y_pid.SetSetpoint(setpoint);

 }

 void t34::CMD_Drop_Back::Execute() 
 {
   auto rc = RobotContainer::get();

   rc->p_grip_solenoid->Set(true);

    rc->m_wrist_y.Set(rc->wrist_y_pid.Calculate(rc->wrist_y_degrees, 0.0));
    rc->m_arm.Set(ControlMode::Position, rc->arm_y_pid.Calculate(rc->m_arm_abs_encoder.GetAbsolutePosition()));

    if (rc->arm_degrees <= 96.0 && placement == "TOP CUBE") {
        rc->p_grip_solenoid->Set(false);
        rc->arm_y_pid.SetSetpoint(320.0);
    }
 }

 void t34::CMD_Drop_Back::End(bool interrupted) {}

bool t34::CMD_Drop_Back::IsFinished() 
{
   return false;
}
