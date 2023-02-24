// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>


Robot::Robot()
{}

void Robot::RobotInit() 
{
  RobotContainer::initialize();
  frc2::CommandScheduler::GetInstance().SetDefaultCommand(RobotContainer::m_drive.get(), *RobotContainer::m_default_command.get());

  RobotContainer::arm_encoder->Reset();
  RobotContainer::wrist_y_encoder->Reset();
  RobotContainer::m_wrist_y->SetNeutralMode(NeutralMode::Coast);
  RobotContainer::m_arm->SetNeutralMode(NeutralMode::Brake);
  RobotContainer::m_wrist_rot->SetNeutralMode(NeutralMode::Brake);

  RobotContainer::wrist_y_pid.SetTolerance(2, 3);
  RobotContainer::wrist_rot_pid.SetTolerance(2, 3);
  RobotContainer::wrist_y_encoder->SetDistancePerPulse(360.0 / 44.4);

  RobotContainer::p_grip_solenoid->Set(0);
  RobotContainer::p_grip_compressor->Enabled();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  //  Converting the encoder values of the arm motor and wrist_Y encoder to degrees
  RobotContainer::wrist_degrees = t34::EncoderToDegree(44.4, RobotContainer::wrist_y_encoder->Get());
  RobotContainer::arm_degrees = t34::EncoderToDegree(4096.0, RobotContainer::arm_encoder->Get());
  RobotContainer::correction_val = t34::CorrectionValue(0.0, RobotContainer::wrist_degrees) * -1.0;
  //frc::SmartDashboard::PutNumber("W Distance per pulse", RobotContainer::wrist_y_encoder->GetDistancePerPulse());
  //frc::SmartDashboard::PutNumber("W Current Position", RobotContainer::wrist_degrees);
  //frc::SmartDashboard::PutNumber("A Distance per pulse", RobotContainer::arm_encoder->GetDistancePerPulse());
  //frc::SmartDashboard::PutNumber("A Current Position", RobotContainer::arm_degrees);
  
  //  Setting the speed of the wrist_Y motor based on PID 
  //RobotContainer::m_wrist_y->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, RobotContainer::wrist_y_pid.Calculate(RobotContainer::correction_val) * -0.2);

  frc::SmartDashboard::PutNumber("W Motor Current", RobotContainer::m_wrist_y->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Wrist Pitch (degrees)", RobotContainer::wrist_degrees);
  frc::SmartDashboard::PutNumber("Wrist Rot", RobotContainer::wrist_rot_encoder->Get());
  frc::SmartDashboard::PutNumber("Correction Val", RobotContainer::correction_val);
  frc::SmartDashboard::PutNumber("Wrist Pitch PID Setpoint", RobotContainer::wrist_y_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("Wrist PID calc", RobotContainer::wrist_y_pid.Calculate(RobotContainer::wrist_degrees, -90.0));
  frc::SmartDashboard::PutNumber("Wrist Rotation PID Setpoint", RobotContainer::wrist_rot_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("Encoder Distance", RobotContainer::wrist_y_encoder->GetDistance());
  frc::SmartDashboard::PutNumber("Wrist Pitch Current", RobotContainer::m_wrist_y->GetOutputCurrent());
  //RobotContainer::m_wrist_y->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, RobotContainer::wrist_y_pid.Calculate(RobotContainer::wrist_degrees, -90.0) * -0.4);
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() 
{}

void Robot::DisabledPeriodic() 
{}

void Robot::AutonomousInit() 
{}

void Robot::AutonomousPeriodic() 
{}

void Robot::TeleopInit() 
{
  RobotContainer::m_drive->zeroYaw();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
  double leftstick_Y = RobotContainer::m_driver_control->getLeftStickYDB();
  double rightstick_X = RobotContainer::m_driver_control->getRightStickXDB();

  frc::SmartDashboard::PutBoolean("Pneumatics Running", RobotContainer::pneumatics_running);
  frc::SmartDashboard::PutBoolean("Drive Brake On", RobotContainer::pneumatics_running);

  //  Wrist Pitch Control, ctre, Right Stick Y
  if (leftstick_Y)
    RobotContainer::m_wrist_y->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.4);


  //  Wrist Rotation Control, ctre, Bumpers
  /*if (RobotContainer::m_driver_control->GetRightBumperPressed())
    RobotContainer::m_wrist_rot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);*/

  /*if (RobotContainer::m_driver_control->GetLeftBumperPressed())
    RobotContainer::m_wrist_rot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.2);*/

  //  Arm Extension Control, rev, D-Pad
  int pov = RobotContainer::m_driver_control->GetPOV();

  if (pov == 0)
    RobotContainer::m_arm_ext->Set(0.2);

  else if (pov == 180)
    RobotContainer::m_arm_ext->Set(-0.2);

  else 
    RobotContainer::m_arm_ext->Set(0.0);

  //  Grip Solenoid, Button X
  if (RobotContainer::m_driver_control->GetXButtonReleased())
    RobotContainer::p_grip_solenoid->Toggle();
  
  //  SheildWall & Toggle Drive Break, Button Select
  if (RobotContainer::m_driver_control->GetBackButtonReleased())
  {
    RobotContainer::m_drive->sheildWall();
    RobotContainer::m_drive->toggleDriveBrake();
  }

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() 
{}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() 
{}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() 
{}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
