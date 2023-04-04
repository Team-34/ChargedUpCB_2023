
 #include "commands/CMD_Drop_Back.h"
 #include "RobotContainer.h"
 #include "Constants.h"

 t34::CMD_Drop_Back::CMD_Drop_Back(std::string placement) :
    placement(placement)
    , setpoint(110.0)
    , arm_drive()
    {}

 void t34::CMD_Drop_Back::Initialize() {
    auto rc = RobotContainer::get();
    arm_drive = rc->armSub.m_arm_abs_encoder.GetAbsolutePosition();

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
   static double speed = 0.7;
   //rc->armSub.p_grip_solenoid->Toggle();

	auto pos = rc->armSub.m_arm_abs_encoder.GetDegrees();
	if (pos < 180.0)
		speed = 0.2;

	
	rc->armSub.m_arm.Set(ControlMode::PercentOutput, -std::clamp(rc->armSub.arm_y_pid.Calculate(pos), -speed, speed));

	if (!rc->armSub.arm_y_pid.AtSetpoint())
	{
	  	rc->armSub.arm_y_pid.SetSetpoint(arm_drive);
	  	arm_drive--;
	}

	if (rc->armSub.m_arm_abs_encoder.GetDegrees() <= 110.0)
	{
	  	arm_drive = 110.0;
		rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), -40.0), -0.5, 0.5));
	}

	if (rc->armSub.wrist_y_pid.AtSetpoint() && rc->armSub.wrist_y_encoder.GetPosition() <= -38.0)
		rc->armSub.p_grip_solenoid->Set(true); 
	
 }

 void t34::CMD_Drop_Back::End(bool interrupted) {}

bool t34::CMD_Drop_Back::IsFinished() 
{
   return false;
}
