// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <algorithm>

#include "commands/CMD_AutoBalance.h"
#include "commands/CMD_DefaultDrive.h"
#include "commands/CMD_Drop_Back.h"


Robot::Robot() {}


void Robot::RobotInit() 
{
  auto rc = RobotContainer::get();

  rc->m_chooser.SetDefaultOption(t34::kAutoNameDefault, t34::kAutoNameDefault);
  rc->m_chooser.AddOption("TOP CUBE", "TOP CUBE");
  rc->m_chooser.AddOption("MID CUBE", "MID CUBE");

  frc::SmartDashboard::PutData("Auto Modes", &rc->m_chooser);

  rc->m_drive->resetOdometer();

  rc->m_wrist_y.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rc->m_wrist_rot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rc->m_arm.SetNeutralMode(NeutralMode::Brake);
  rc->m_arm_ext.SetNeutralMode(Coast);

  rc->m_arm_abs_encoder.Reset();
  rc->m_arm_abs_encoder.SetDistancePerRotation(2048.0 * 32.0);

  rc->wrist_y_pid.SetTolerance(2, 3);
  rc->wrist_rot_pid.SetTolerance(2, 3);
  rc->wrist_y_encoder.SetPositionConversionFactor(1.0);
  rc->wrist_rot_encoder.SetPositionConversionFactor(1.0);
  rc->wrist_y_encoder.SetPosition(-24.0);
  rc->wrist_rot_encoder.SetPosition(0.0);
  
  rc->p_grip_solenoid->Set(0);
  rc->p_grip_compressor->Enabled();

  frc2::CommandScheduler::GetInstance().SetDefaultCommand(rc->m_drive.get(), rc->m_default_command);
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

  frc::SmartDashboard::PutData(&rc->m_chooser);

  //rc->targetOffsetAngle_Horizontal = rc->table->GetNumber("tx",0.0);
  //rc->targetOffsetAngle_Vertical = rc->table->GetNumber("ty",0.0);
  //rc->targetArea = rc->table->GetNumber("ta",0.0);
  //rc->targetSkew = rc->table->GetNumber("ts",0.0);

 /*
 //   Output Drive Encoder to SmartDashBoard
  frc::SmartDashboard::PutNumber("Encoder Val, LA", rc->m_drive->m_la.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, LF", rc->m_drive->m_lf.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, RA", rc->m_drive->m_ra.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, RF", rc->m_drive->m_rf.encoder.GetAbsolutePosition());
  */
  frc::SmartDashboard::PutNumber("Odometer", rc->m_drive->getOdometer());
  frc::SmartDashboard::PutNumber("NavX Pitch", rc->navX.GetPitch());
/*
//    Arm pitch and extention limit switches
  frc::SmartDashboard::PutBoolean("Arm Ext Fwd Limit", rc->m_arm_ext.IsFwdLimitSwitchClosed());
  frc::SmartDashboard::PutBoolean("Arm Ext Rev Limit", rc->m_arm_ext.IsRevLimitSwitchClosed());
  frc::SmartDashboard::PutBoolean("Arm Fwd Limit", rc->m_limit_switch_front.Get());
  frc::SmartDashboard::PutBoolean("Arm Rev Limit", rc->m_limit_switch_back.Get());
*/
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
{

  auto rc = RobotContainer::get();
  rc->m_drive->zeroYaw();
  rc->m_drive->resetOdometer();
  rc->m_drive->setDriveMode(t34::DriveMode::FieldOriented);

  frc2::CommandScheduler::GetInstance().Schedule(new frc2::SequentialCommandGroup(
    t34::CMD_Drop_Back(rc->m_chooser.GetSelected()),
    t34::CMD_DriveStraightDistance(133.0 * t34::UNITS_PER_INCH, 0.2, 0.0)
  ));
}

void Robot::AutonomousPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TeleopInit() 
{
  auto rc = RobotContainer::get();
  auto sc = rc->m_arm.GetSensorCollection();
  
  rc->m_drive->setDriveMode(t34::DriveMode::FieldOriented);
  sc.SetIntegratedSensorPosition(30.0);
  rc->arm_y_pid.SetSetpoint(300.0);
  rc->wristTog = false;

}

/*
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
  auto rc = RobotContainer::get();

  auto armdegrees = rc->m_arm_abs_encoder.GetDegrees();

  auto w_rotation = (((rc->wrist_rot_encoder.GetPosition() * 100.0) / 360.0));
  auto w_pitch = (((rc->wrist_y_encoder.GetPosition() * 70.0 ) / 360.0));

  double rightstick_y = rc->m_driver_control->getRightStickYDB();
  double rightstick_x = rc->m_driver_control->getRightStickXDB();

  double w_degrees = rc->wrist_y_encoder.GetPosition();
  double w_rot_degrees = rc->wrist_rot_encoder.GetPosition();
  int pov = rc->m_driver_control->GetPOV();

  double armpidout = 320.0;
  double wrpidout = 0.0;

  frc::SmartDashboard::PutNumber("Encoder Counts", rc->wrist_y_encoder.GetCountsPerRevolution());
  frc::SmartDashboard::PutNumber("Encoder Cvt Factor", rc->wrist_y_encoder.GetPositionConversionFactor());
  frc::SmartDashboard::PutNumber("Wrist Y Encoder Units", rc->wrist_y_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Wrist Rot Encoder Units", rc->wrist_rot_encoder.GetPosition());

  frc::SmartDashboard::PutNumber("Wrist Pitch", w_pitch);
  frc::SmartDashboard::PutNumber("Wrist Rotation", w_rotation);

  frc::SmartDashboard::PutBoolean("Front LS", rc->m_limit_switch_front.Get());
  frc::SmartDashboard::PutBoolean("Back LS", rc->m_limit_switch_back.Get());

  frc::SmartDashboard::PutNumber("arm pitch", armdegrees);
  frc::SmartDashboard::PutNumber("arm sp", rc->arm_y_pid.GetSetpoint());


  //  Wrist Control, D-pad
  switch (pov)
  {
    case 0:
      w_degrees++;
      break;

    case 180:
      w_degrees--;
      break;
    
    case 90:
      w_rot_degrees++;
      break;
    case 270:
      w_rot_degrees--;
      break;
  }

  if (w_degrees < -90.0)
    w_degrees = -90.0;
  if (w_degrees > 0.0)
    w_degrees = 0.0;

  frc::SmartDashboard::PutNumber("w_deg", w_degrees);
  frc::SmartDashboard::PutNumber("w_rot_deg", w_rot_degrees);

  rc->m_wrist_y.Set(std::clamp(rc->wrist_y_pid.Calculate(rc->wrist_y_encoder.GetPosition(), w_degrees), -1.0, 1.0));
  rc->m_wrist_rot.Set(std::clamp(rc->wrist_rot_pid.Calculate(rc->wrist_rot_encoder.GetPosition(), w_rot_degrees), -1.0, 1.0));


  //  Grip Solenoid, Button X
  if (rc->m_driver_control->GetXButtonReleased())
    rc->p_grip_solenoid->Toggle();


  //  ZERO YAW, Y
  if (rc->m_driver_control->GetYButtonReleased())
    rc->m_drive->zeroYaw();
  

  //  SheildWall & Toggle Drive Break, Button Select
  if (rc->m_driver_control->GetBackButtonReleased())
    rc->m_drive->sheildWall();


  //    Arm Control
  if (rc->m_driver_control->GetRightBumper()) {
    if (rc->m_limit_switch_back.Get()) {
          auto current = rc->arm_y_pid.GetSetpoint();

          if (current < 44.0) {
            rc->arm_y_pid.SetSetpoint(44.0);
          }

          else {
            rc->arm_y_pid.SetSetpoint( current - t34::ARM_PITCH_VAL);
          }
    }

  }
  else if (rc->m_driver_control->GetLeftBumper()) {
      if (rc->m_limit_switch_front.Get()) {
          auto current = rc->arm_y_pid.GetSetpoint();

          if (current > 320.0) 
          {
             rc->arm_y_pid.SetSetpoint(320.0);
          }

          else {
            rc->arm_y_pid.SetSetpoint( current + t34::ARM_PITCH_VAL);
          }

      }
  }
  rc->m_arm.Set(ControlMode::PercentOutput, -std::clamp(rc->arm_y_pid.Calculate(armdegrees), -0.5, 0.5));


  //  ARM EXT CONTROL
  rc->m_arm_ext.Set(ControlMode::PercentOutput, -rightstick_y);
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
