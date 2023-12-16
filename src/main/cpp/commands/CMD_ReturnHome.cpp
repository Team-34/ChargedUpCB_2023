// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CMD_ReturnHome.h"

namespace t34
{
	CMD_ReturnHome::CMD_ReturnHome()
	//: arm_drive()
	{}
	
	// Called when the command is initially scheduled.
	void CMD_ReturnHome::Initialize() {
	  auto rc = RobotContainer::get();
	
		//arm_drive = rc->armSub.m_arm_abs_encoder.GetAbsolutePosition();
		rc->armSub.arm_y_pid.SetSetpoint(337.0);
		rc->armSub.wrist_y_pid.SetSetpoint(0.0);
		rc->armSub.ext_sp = 0.0;
		rc->armSub.arm_ext_pid.SetSetpoint(rc->armSub.ext_sp);
		

		rc->armSub.m_arm_ext.SetNeutralMode(NeutralMode::Coast);
		frc::SmartDashboard::PutBoolean("return home", true);
	}
	
	// Called repeatedly when this Command is scheduled to run
	void CMD_ReturnHome::Execute() {
	  auto rc = RobotContainer::get();

	  //rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), 0.0), -1.0, 1.0));
	  if (rc->armSub.wrist_y_encoder.GetPosition() < -1.5)
	  	rc->armSub.m_wrist_y.Set(0.5);
	  else 
	  	rc->armSub.m_wrist_y.Set(0.0);

     rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, std::clamp(rc->armSub.arm_ext_pid.Calculate(rc->armSub.arm_ext_encoder.GetDistance(), 
                                                                                                  rc->armSub.ext_sp), -0.6, 0.6));
    if (rc->armSub.arm_ext_encoder.GetDistance() != 0.0)
    	rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, -0.5);
    else 
    	rc->armSub.ArmExtZero();
	  
	  double out = std::clamp(rc->armSub.arm_y_pid.Calculate(rc->armSub.m_arm_abs_encoder.GetDegrees()), -0.5, 0.5);
	  if (out > 0.0) out *= -1.0;

	  //if (rc->armSub.arm_ext_encoder.GetDistance() >= -200.0)
		rc->armSub.m_arm.Set(ControlMode::PercentOutput, out);

		frc::SmartDashboard::PutNumber("Out", out);
	
	
	//   if (rc->armSub.wrist_y_pid.AtSetpoint())
	//   {
	//     rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, rc->armSub.arm_ext_pid.Calculate(rc->armSub.arm_ext_encoder.Get()));
	//   }
	
	//   if (rc->armSub.arm_ext_pid.AtSetpoint())
	//   {
	//     rc->armSub.m_arm.Set(ControlMode::Position, rc->armSub.arm_y_pid.Calculate(rc->armSub.m_arm_abs_encoder.GetAbsolutePosition()));
	//   }
		return;
	}
	
	// Called once the command ends or is interrupted.
	void CMD_ReturnHome::End(bool interrupted) {
	//  auto rc = RobotContainer::get();
	
	//   rc->armSub.m_arm.Set(ControlMode::PercentOutput, 0.0);
	//   rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, 0.0);
	//   rc->armSub.m_wrist_y.Set(0.0);
	  return;
	}
	
	// Returns true when the command should end.
	bool CMD_ReturnHome::IsFinished() {
	  auto rc = RobotContainer::get();

	  if (rc->armSub.m_arm_abs_encoder.GetDegrees() >= 320.0)
	  {
	  	rc->armSub.m_arm.Set(ControlMode::PercentOutput, 0.0);
		rc->armSub.m_arm_ext.SetNeutralMode(NeutralMode::Brake);
		frc::SmartDashboard::PutBoolean("Return Complete", true);
	  	return true;
	  }
	  else
	  	return false;
	}
}
