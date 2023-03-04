// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "commands/CMD_DefaultDrive.h"

Robot::Robot() {}


void Robot::RobotInit() 
{
  auto rc = RobotContainer::get();
  //g_def_cmd = t34::DefaultDriveCommand(rc->m_drive, rc->m_driver_control);
  frc2::CommandScheduler::GetInstance().SetDefaultCommand(rc->m_drive.get(), rc->m_default_command);

  rc->wrist_y_encoder.Reset();
  rc->m_wrist_y.SetNeutralMode(NeutralMode::Brake);
  rc->m_wrist_rot.SetNeutralMode(NeutralMode::Brake);
  rc->m_arm->SetNeutralMode(NeutralMode::Brake);

  rc->arm_encoder.Reset();
  rc->arm_encoder.SetDistancePerRotation(2048.0 * 32.0);

  rc->m_arm_ext.SetCANTimeout(200);
  rc->m_arm_ext.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  rc->wrist_y_pid.SetTolerance(2, 3);
  rc->wrist_rot_pid.SetTolerance(2, 3);
  //rc->wrist_y_encoder.SetDistancePerPulse(360.0 / 44.4);

  rc->p_grip_solenoid->Set(0);
  rc->p_grip_compressor->Disable();
  //rc->arm_encoder->SetDistancePerRotation(1024.0 / 360.0);

  rc->wristRotTog = false;
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
  frc2::CommandScheduler::GetInstance().Run();
  
  auto rc = RobotContainer::get();
  //  Converting the encoder values of the arm motor and wrist_Y encoder to degrees
  rc->wrist_degrees = t34::EncoderToDegree(44.4, rc->wrist_y_encoder.Get());
  //rc->arm_degrees = t34::EncoderToDegree(4096.0, rc->arm_encoder->GetAbsolutePosition());
  rc->correction_val = t34::CorrectionValue(0.0, rc->wrist_degrees) * -1.0;
  //frc::SmartDashboard::PutNumber("W Distance per pulse", rc->wrist_y_encoder->GetDistancePerPulse());
  //frc::SmartDashboard::PutNumber("W Current Position", rc->wrist_degrees);
  //frc::SmartDashboard::PutNumber("A Distance per pulse", rc->arm_encoder->GetDistancePerPulse());
  //frc::SmartDashboard::PutNumber("A Current Position", rc->arm_degrees);
  
  //  Setting the speed of the wrist_Y motor based on PID 
  //rc->m_wrist_y->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rc->wrist_y_pid.Calculate(rc->correction_val) * -0.2);
  /*
  frc::SmartDashboard::PutNumber("W Motor Current", rc->m_wrist_y->GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Wrist Pitch (degrees)", rc->wrist_degrees);
  frc::SmartDashboard::PutNumber("Wrist Rot", rc->wrist_rot_encoder->Get());
  frc::SmartDashboard::PutNumber("Correction Val", rc->correction_val);
  frc::SmartDashboard::PutNumber("Wrist Pitch PID Setpoint", rc->wrist_y_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("Wrist PID calc", rc->wrist_y_pid.Calculate(rc->wrist_degrees, -90.0));
  frc::SmartDashboard::PutNumber("Wrist Rotation PID Setpoint", rc->wrist_rot_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("Encoder Distance", rc->wrist_y_encoder->GetDistance());
  frc::SmartDashboard::PutNumber("Wrist Pitch Current", rc->m_wrist_y->GetOutputCurrent());
  
*/
  frc::SmartDashboard::PutNumber("Encoder Val, LA", rc->m_drive->m_la.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, LF", rc->m_drive->m_lf.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, RA", rc->m_drive->m_ra.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, RF", rc->m_drive->m_rf.encoder.GetAbsolutePosition());
  
 //  frc::SmartDashboard::PutNumber("Encoder Val, RF", cc.GetPosition());

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
  auto rc = RobotContainer::get();
  rc->m_drive->zeroYaw();
}

/**
 * This function is called periodically during operator control.
 */
constexpr double arm_full_units{ 2048.0 * 32.0 };
void Robot::TeleopPeriodic() 
{
  auto rc = RobotContainer::get();
  bool wristTog = rc->wristRotTog;
  double rightstick_y = rc->m_driver_control->getRightStickYDB();
  double rightstick_x = rc->m_driver_control->getRightStickXDB();

  // auto sc = rc->m_arm->GetSensorCollection();
  // auto enc = sc.GetIntegratedSensorAbsolutePosition()  * 32.0;
  //auto abs_enc = rc->arm_encoder.GetVoltage();

  frc::SmartDashboard::PutNumber("Wrist Rotation", rc->wrist_rot_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("Wrist Pitch", rc->wrist_y_encoder.GetDistance());
  //frc::SmartDashboard::PutNumber("Arm Pitch Encoder", abs_enc);//rc->arm_encoder->GetAbsolutePosition());
  //frc::SmartDashboard::PutNumber("Arm Pitch Encoder m", ((360.0 / 5.0) * abs_enc));
  frc::SmartDashboard::PutNumber("Arm Analog Encoder Degrees", rc->arm_encoder.GetAbsolutePosition() * 360.0);
  frc::SmartDashboard::PutNumber("Arm Analog Encoder", rc->arm_encoder.GetAbsolutePosition());
  //frc::SmartDashboard::PutNumber("Arm Pitch Encoder Acc count", rc->arm_encoder.GetAccumulatorCount());

//  int pov = rc->m_driver_control->GetPOV();
//  double arm_pitch_setpoint = 180.0 * rightstick_y;

  //  Toggle Wrist Rotation With Bumper
  if (rc->m_driver_control->GetRightBumperReleased())
    wristTog = !wristTog;

  if (wristTog == true)
    rc->m_wrist_rot.Set(ControlMode::Position, rc->wrist_rot_pid.Calculate(180.0));
  if (wristTog == false)
    rc->m_wrist_rot.Set(ControlMode::Position, rc->wrist_rot_pid.Calculate(0.0));

  //  Arm Pitch Control, ctre, Right Stick Y
  rc->m_arm->Set(ControlMode::PercentOutput, rightstick_y);
  rc->m_wrist_rot.Set(ControlMode::PercentOutput, rightstick_x);

  // if (rc->m_driver_control->GetLeftBumperPressed())
  //   rc->m_wrist_rot.Set(ControlMode::PercentOutput, 0.7);
  // else if (rc->m_driver_control->GetRightBumperPressed())
  //   rc->m_wrist_rot.Set(ControlMode::PercentOutput, 0.7);
  // else
  //   rc->m_wrist_rot.Set(ControlMode::PercentOutput, 0.0);

//     rc->m_arm->Set(ControlMode::PercentOutput, rightstick_y * 0.4);

//frc::SmartDashboard::PutNumber("Motor Current", rc->m_arm_ext.GetOutputCurrent());
//frc::SmartDashboard::PutNumber("Right stick y", rightstick_y);
    //rc->m_arm->Set(ControlMode::Position, arm_pitch_setpoint);


  //  Wrist Pitch Control, D-Pad left & right
   

//   if (pov == 0)
//     rc->m_wrist_y->Set(ControlMode::PercentOutput, 0.2);

//   else if (pov == 180)
//     rc->m_wrist_y->Set(ControlMode::PercentOutput, -0.2);

//   else 
//     rc->m_wrist_y->Set(ControlMode::PercentOutput, 0.0);


  //  Arm Extension Control, D-Pad up & down
//   if (pov == 270)
//     rc->m_arm_ext->Set(0.2);

//   else if (pov == 90)
//     rc->m_arm_ext->Set(-0.2);

//   else 
//     rc->m_arm_ext->Set(0.0);


  //  Grip Solenoid, Button X
 if (rc->m_driver_control->GetXButtonReleased())
   rc->p_grip_solenoid->Toggle();
  

  //  SheildWall & Toggle Drive Break, Button Select
//  if (rc->m_driver_control->GetBackButtonReleased())
//    rc->m_drive->sheildWall();
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
