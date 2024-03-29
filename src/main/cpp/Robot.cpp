// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <algorithm>

#include "commands/CMD_AutoBalance.h"
#include "commands/CMD_DefaultDrive.h"
#include "commands/CMD_Drop_Back.h"
#include "cameraserver/CameraServer.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

void vision() {
  using namespace std::literals;
  std::this_thread::sleep_for(std::chrono::duration(1s));
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  
  camera.SetResolution(320, 240);
//  cs::CvSink sink = frc::CameraServer::GetVideo();
//  cs::CvSource stream = frc::CameraServer::PutVideo("Main", 320, 240);
//  cv::Mat source;

  while (true) {
//    sink.GrabFrame(source);
//    stream.PutFrame(source);

  }

}

Robot::Robot() {}


void Robot::RobotInit() 
{
  auto rc = RobotContainer::get();
  //frc::CameraServer::StartAutomaticCapture(0);
  //cs::CvSink cvSink = frc::CameraServer::GetVideo();
  
  std::thread vision_thread(vision);
  vision_thread.detach();

  rc->m_chooser.SetDefaultOption(t34::kAutoNameDefault, t34::kAutoNameDefault);
  rc->m_chooser.AddOption("TOP CUBE", "TOP CUBE");
  rc->m_chooser.AddOption("MID CUBE", "MID CUBE");
  rc->m_chooser.AddOption("DeployAndDrive", "DeployAndDrive");
  rc->m_chooser.AddOption("DeployAndBalance", "DeployAndBalance");

  frc::SmartDashboard::PutData("Auto Modes", &rc->m_chooser);

  rc->m_drive->resetOdometer();
  rc->m_drive->setDriveBrake(true);

  rc->armSub.p_grip_compressor->Enabled();

  rc->armSub.arm_ext_encoder.Reset();

  frc2::CommandScheduler::GetInstance().SetDefaultCommand(rc->m_drive.get(), rc->m_default_command);
  //rc->m_drive->zeroIntegratedEncoders();
  
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

  arm_degrees = rc->armSub.m_arm_abs_encoder.GetDegrees();
  frc::SmartDashboard::PutNumber("wrist current", rc->armSub.m_wrist_y.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Arm degrees", rc->armSub.m_arm_abs_encoder.GetDegrees());
  frc::SmartDashboard::PutNumber("ARM SP", rc->armSub.arm_y_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("Wrist Enc", rc->armSub.wrist_y_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Wrist SP", rc->armSub.wrist_y_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("Arm Ext enc", rc->armSub.arm_ext_encoder.GetDistance());
  frc::SmartDashboard::PutBoolean("Solenoid", rc->armSub.p_grip_solenoid->Get());
  frc::SmartDashboard::PutBoolean("Back LS", rc->m_limit_switch_back.Get());

  frc::SmartDashboard::PutNumber("NavX Pitch", rc->navX.GetPitch());
  frc::SmartDashboard::PutNumber("NavX Yaw", rc->navX.GetYaw());
  frc::SmartDashboard::PutNumber("NavX Roll", rc->navX.GetRoll());


  frc::SmartDashboard::PutNumber("LF ENC", rc->m_drive->m_lf->encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("RF ENC", rc->m_drive->m_rf->encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("LA ENC", rc->m_drive->m_la->encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("RA ENC", rc->m_drive->m_ra->encoder.GetAbsolutePosition());

  
  //cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur", 640, 480);
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() 
{
  auto rc = RobotContainer::get();
  rc->m_drive->setDriveBrake(false);
  rc->armSub.m_arm.SetNeutralMode(NeutralMode::Coast);
  
}

void Robot::DisabledPeriodic() 
{}

void Robot::AutonomousInit() 
{
  auto rc = RobotContainer::get();
  rc->m_drive->zeroYaw();
  rc->m_drive->resetOdometer();
  rc->m_drive->setDriveMode(t34::DriveMode::FieldOriented);
  rc->armSub.wrist_y_encoder.SetPosition(0.0);

  rc->armSub.p_grip_solenoid->Set(false);

  if (rc->m_chooser.GetSelected() == "DeployAndDrive") {
    frc2::CommandScheduler::GetInstance().Schedule(new frc2::SequentialCommandGroup(
     t34::CMD_Drop_Back(rc->m_chooser.GetSelected()),
     t34::CMD_ReturnHome(),
     t34::CMD_DriveStraightDistance(140.0 * t34::UNITS_PER_INCH, 0.7, 0.0)
    ));    
  }
  else if (rc->m_chooser.GetSelected() == "DeployAndBalance") {
    frc2::CommandScheduler::GetInstance().Schedule(new frc2::SequentialCommandGroup(
    t34::CMD_Drop_Back(rc->m_chooser.GetSelected()),
    t34::CMD_ReturnHome(),
    t34::CMD_AutoBal()
    ));
  }
  else {
    frc2::CommandScheduler::GetInstance().Schedule(new frc2::SequentialCommandGroup(
    t34::CMD_Drop_Back(rc->m_chooser.GetSelected()),
    t34::CMD_ReturnHome()
///    t34::CMD_AutoBal()
    ));    
  }
  
}

void Robot::AutonomousPeriodic() 
{
 // auto rc = RobotContainer::get();

  //rc->armSub.ArmExtZero();

  //frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TeleopInit() 
{
  auto rc = RobotContainer::get();
  auto sc = rc->armSub.m_arm.GetSensorCollection();

  rc->armSub.wrist_y_encoder.SetPosition(0.0);
  rc->m_drive->setDriveMode(t34::DriveMode::FieldOriented);
  sc.SetIntegratedSensorPosition(30.0);
  rc->armSub.arm_y_pid.SetSetpoint(337.0);
  rc->zeroed_steer = false;
  rc->armSub.p_grip_solenoid->Set(false);
}

/*
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
  auto rc = RobotContainer::get();

  auto w_rotation = (((rc->armSub.wrist_rot_encoder.GetPosition() * 100.0) / 360.0));
  auto w_pitch = (((rc->armSub.wrist_y_encoder.GetPosition() * 70.0 ) / 360.0));

  double rightstick_y = rc->m_driver_control->getRightStickYDB();
  double rightstick_x = rc->m_driver_control->getRightStickXDB();

  double w_degrees = rc->armSub.wrist_y_encoder.GetPosition();
  double w_rot_degrees = rc->armSub.wrist_rot_encoder.GetPosition();

  
  frc::SmartDashboard::PutNumber("Wrist Pitch", w_pitch);
  frc::SmartDashboard::PutNumber("Wrist Rotation", w_rotation);


  rc->armSub.ArmExtZero();
  

  //  Wrist Control, D-pad
  int pov = rc->m_driver_control->GetPOV();
  
  switch (pov)
    {
      case 0:
        w_degrees = w_degrees + 5;
        break;

      case 180:
        w_degrees = w_degrees - 5;
        break;

      case 90:
        w_rot_degrees = w_rot_degrees + 5;
        break;
      case 270:
        w_rot_degrees = w_rot_degrees - 5;
        break;
    }
  rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), w_degrees), -1.0, 1.0));
  rc->armSub.m_wrist_rot.Set(std::clamp(rc->armSub.wrist_rot_pid.Calculate(rc->armSub.wrist_rot_encoder.GetPosition(), w_rot_degrees), -1.0, 1.0));

  // if (w_degrees < -90.0)
  //   w_degrees = -90.0;
  // if (w_degrees > 0.0)
  //   w_degrees = 0.0;

  frc::SmartDashboard::PutNumber("w_deg", w_degrees);
  frc::SmartDashboard::PutNumber("w_rot_deg", w_rot_degrees);

  rc->armSub.m_wrist_y.Set(std::clamp(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_encoder.GetPosition(), w_degrees), -1.0, 1.0));
  rc->armSub.m_wrist_rot.Set(std::clamp(rc->armSub.wrist_rot_pid.Calculate(rc->armSub.wrist_rot_encoder.GetPosition(), w_rot_degrees), -1.0, 1.0));


  //  Grip Solenoid, Button X
  if (rc->m_driver_control->GetXButtonReleased())
    rc->armSub.p_grip_solenoid->Toggle();


  //  ZERO YAW, Y
  if (rc->m_driver_control->GetYButtonReleased())
    rc->m_drive->zeroYaw();
  

  //  SheildWall & Toggle Drive Break, Button Select
  if (rc->m_driver_control->GetBackButtonReleased())
    rc->m_drive->shieldWall();


  //    Arm Control, Bumpers: L down  R up
  rc->armSub.Arm_Pitch_Cntrl();
  rc->armSub.m_arm.Set(ControlMode::PercentOutput, -std::clamp(rc->armSub.arm_y_pid.Calculate(arm_degrees), -0.3, 0.3));



  //  ARM EXT CONTROL, Rigth Stick Y
  auto r_y = -rightstick_y * 0.5;
  if (rc->armSub.arm_ext_encoder.GetDistance() > -1990.0 || r_y < 0.0)
    rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, r_y );
  else
    rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, 0.0); 

  //  Faris Mode Toggle, A button
  if (rc->m_driver_control->GetAButton())
  {
    rc->m_drive->toggleSpeed();
  }


  //  Return Home Arm, Start Button
  if (rc->m_driver_control->GetStartButton())
  {
    frc2::CommandScheduler::GetInstance().Schedule(new t34::CMD_ReturnHome());
  }

  if (rc->m_driver_control->GetLeftStickButtonReleased())
  {
    if(rc->armSub.p_grip_compressor->IsEnabled())
    {
      rc->armSub.p_grip_compressor->Disable();
    }
    else
    {
      rc->armSub.p_grip_compressor->Enabled();  
    }
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

//  TODO: Fix swerve jank, correct arm's fall during rotation, organize code