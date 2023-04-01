
 #include "commands/CMD_Drop_Back.h"
 #include "RobotContainer.h"
 #include "Constants.h"

 t34::CMD_Drop_Back::CMD_Drop_Back(std::string placement) :
    placement(placement)
    , setpoint(96.0){}

 void t34::CMD_Drop_Back::Initialize() {
    auto rc = RobotContainer::get();

    rc->armSub.p_grip_solenoid->Set(true);

    if (placement == "TOP CUBE") {
        setpoint = 96.0;
    }

    else if (placement == "MID CUBE") {
        setpoint = 86.0;
    }

    rc->armSub.arm_y_pid.SetSetpoint(setpoint);

    return;
 }

 void t34::CMD_Drop_Back::Execute() 
 {
   auto rc = RobotContainer::get();
   double w_degrees = rc->armSub.wrist_y_encoder.GetPosition();

    rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), -20.0), -1.0, 1.0));
    rc->armSub.m_arm.Set(ControlMode::PercentOutput, -std::clamp(rc->armSub.arm_y_pid.Calculate(rc->armSub.m_arm_abs_encoder.GetDegrees()), -0.2, 0.2));

    if (rc->armSub.arm_y_pid.AtSetpoint())
    {
        rc->armSub.p_grip_solenoid->Set(false);
    }
 }

 void t34::CMD_Drop_Back::End(bool interrupted) {}

bool t34::CMD_Drop_Back::IsFinished() 
{
   return false;
}
