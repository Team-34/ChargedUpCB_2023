// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CMD_ReturnHome.h"

namespace t34
{
	
	CMD_ReturnHome::CMD_ReturnHome() {
	  // Use addRequirements() here to declare subsystem dependencies.
	}
	
	// Called when the command is initially scheduled.
	void CMD_ReturnHome::Initialize() {
	  auto rc = RobotContainer::get();
	
	  rc->armSub.arm_y_pid.SetSetpoint(333.0);
	  rc->armSub.wrist_y_pid.SetSetpoint(0.0);
	  rc->armSub.arm_ext_pid.SetSetpoint(0.0);
	}
	
	// Called repeatedly when this Command is scheduled to run
	void CMD_ReturnHome::Execute() {
	  auto rc = RobotContainer::get();
	
	  rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), 0.0), -1.0, 1.0));
	
	  if (rc->armSub.wrist_y_pid.AtSetpoint())
	  {
	    rc->armSub.m_arm_ext.Set(ControlMode::Position, rc->armSub.arm_ext_pid.Calculate(rc->armSub.arm_ext_encoder.Get()));
	  }
	
	  if (rc->armSub.arm_ext_pid.AtSetpoint())
	  {
	    rc->armSub.m_arm.Set(ControlMode::Position, rc->armSub.arm_y_pid.Calculate(rc->armSub.m_arm_abs_encoder.GetAbsolutePosition()));
	  }
	}
	
	// Called once the command ends or is interrupted.
	void CMD_ReturnHome::End(bool interrupted) {
	  auto rc = RobotContainer::get();
	
	  rc->armSub.m_arm.Set(ControlMode::PercentOutput, 0.0);
	  rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, 0.0);
	  rc->armSub.m_wrist_y.Set(0.0);
	}
	
	// Returns true when the command should end.
	bool CMD_ReturnHome::IsFinished() {
	  return false;
	}
}
