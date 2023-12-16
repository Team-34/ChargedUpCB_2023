
 #include "commands/CMD_Drop_Back.h"
 #include "RobotContainer.h"
 #include "Constants.h"

#define phase_1         0  // move to 145
#define phase_2         1  // wrist to 40
#define phase_3         2  // move to 120
#define phase_4         3  // release cube

 t34::CMD_Drop_Back::CMD_Drop_Back(std::string placement) :
    placement(placement)
    , setpoint(120.0)
    , arm_drive()
    {}

 void t34::CMD_Drop_Back::Initialize() {
    auto rc = RobotContainer::get();
    timer = 0;
    state = phase_1;
    w_setpoint = 0.0;
    arm_drive = rc->armSub.m_arm_abs_encoder.GetAbsolutePosition();
    //rc->armSub.arm_ext_pid.SetSetpoint(-1775);
    if (placement == "TOP CUBE") {
        setpoint = 120.0;
    }

    rc->armSub.arm_y_pid.SetSetpoint(setpoint);

    return;
 }



 void t34::CMD_Drop_Back::Execute() 
 {
   auto rc = RobotContainer::get();
   double w_degrees = rc->armSub.wrist_y_encoder.GetPosition();
   static double speed = 0.6;

	auto pos = rc->armSub.m_arm_abs_encoder.GetDegrees();
	if (pos < 180.0)
		speed = 0.4;

// 	switch(state)
//    {
//       case phase_1 : 
//          setpoint = 140.0;
//          if (rc->armSub.m_arm_abs_encoder.GetDegrees() < 140.0)
//             state++;
//       break;

//       case phase_2 :
//          w_setpoint = -40.0;
//          if (rc->armSub.wrist_y_pid.AtSetpoint())
//             state++;
//       break;

//       case phase_3 :
//          setpoint = 120.0;
//          if (rc->armSub.m_arm_abs_encoder.GetDegrees() < 120.0)
//          {
//             rc->armSub.p_grip_solenoid->Toggle();
//             state = phase_4;
//          }

//       break;

//    }




// rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), w_setpoint), -0.4, 0.4));
// rc->armSub.m_arm.Set(ControlMode::PercentOutput, -std::clamp(rc->armSub.arm_y_pid.Calculate(pos, setpoint), -speed, speed));




	rc->armSub.m_arm.Set(ControlMode::PercentOutput, -std::clamp(rc->armSub.arm_y_pid.Calculate(pos), -speed, speed));

	if (!rc->armSub.arm_y_pid.AtSetpoint())
	{
	  	rc->armSub.arm_y_pid.SetSetpoint(arm_drive);
	  	arm_drive--;
	}


	if (rc->armSub.m_arm_abs_encoder.GetDegrees() <= 145.0)
	{
	  	arm_drive = 120.0;
		rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), -40.0), -0.4, 0.4));
	}


	if (rc->armSub.wrist_y_pid.AtSetpoint() && rc->armSub.wrist_y_encoder.GetPosition() <= -38.0)
   {
      timer++;
		rc->armSub.p_grip_solenoid->Set(true); 
   }


	
 }

 void t34::CMD_Drop_Back::End(bool interrupted) {
   //frc2::CommandScheduler::GetInstance().Schedule(new t34::CMD_ReturnHome());
   return;
 }

bool t34::CMD_Drop_Back::IsFinished() 
{
   auto rc = RobotContainer::get();
   if (rc->armSub.p_grip_solenoid->Get() == true && timer > 50) {
      frc::SmartDashboard::PutBoolean("Drop Complete", true);
      rc->armSub.m_arm.Set(ControlMode::PercentOutput, 0.0);
      return true;
   }
   // if (state == phase_4) {
   //    frc::SmartDashboard::PutBoolean("Drop Complete", true);
   //    rc->armSub.m_arm.Set(ControlMode::PercentOutput, 0.0);
   //    return true;
   // }
   else {
      frc::SmartDashboard::PutBoolean("Drop Complete", false);
      return false;
   }
}
//Try: Implementing a temporary arm PID for the duration of the command; delete it inside of ::End()->mess with the PID constants
