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

  rc->m_wrist_y.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rc->m_wrist_rot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rc->m_arm.SetNeutralMode(NeutralMode::Brake);
  rc->m_arm_ext.SetNeutralMode(Coast);

  rc->m_arm_abs_encoder.Reset();
  rc->m_arm_abs_encoder.SetDistancePerRotation(2048.0 * 32.0);
  //rc->wrist_y_encoder.SetDistancePerPulse(360.0 / 44.4);
  //rc->arm_encoder->SetDistancePerRotation(1024.0 / 360.0);

  rc->wrist_y_pid.SetTolerance(2, 3);
  rc->wrist_rot_pid.SetTolerance(2, 3);
  
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

  //  Converting the encoder values of the arm motor and wrist_Y encoder to degrees
  rc->wrist_y_degrees = t34::EncoderToDegree(44.4, rc->wrist_y_encoder.GetPosition());
  rc->wrist_rot_degrees = t34::EncoderToDegree(44.4, rc->wrist_rot_encoder.GetPosition());
 
 //   Output Drive Encoder to SmartDashBoard
  frc::SmartDashboard::PutNumber("Encoder Val, LA", rc->m_drive->m_la.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, LF", rc->m_drive->m_lf.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, RA", rc->m_drive->m_ra.encoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Encoder Val, RF", rc->m_drive->m_rf.encoder.GetAbsolutePosition());
  
  frc::SmartDashboard::PutNumber("Odometer", rc->m_drive->getOdometer());

//    Arm pitch and extention limit switches
  frc::SmartDashboard::PutBoolean("Arm Ext Fwd Limit", rc->m_arm_ext.IsFwdLimitSwitchClosed());
  frc::SmartDashboard::PutBoolean("Arm Ext Rev Limit", rc->m_arm_ext.IsRevLimitSwitchClosed());
  frc::SmartDashboard::PutBoolean("Arm Fwd Limit", rc->m_limit_switch_front.Get());
  frc::SmartDashboard::PutBoolean("Arm Rev Limit", rc->m_limit_switch_back.Get());

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

  frc2::CommandScheduler::GetInstance().Schedule(new frc2::SequentialCommandGroup(
    t34::CMD_DriveStraightDistance(5002496.613418532, 0.2, 0.0)
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

  sc.SetIntegratedSensorPosition(30.0);
  //rc->arm_y_pid.SetSetpoint(320.0);
  //rc->wrist_y_pid.SetSetpoint(0.0);
  rc->wristTog = false;
}

inline double getclampedpid(double& armdegrees, double& pidout, RobotContainer* rc){
  pidout = rc->arm_y_pid.Calculate(armdegrees);
  if (pidout > 0.2)
    pidout = 0.2;
  if (pidout < -0.2)
    pidout = -0.2;

    return pidout;
};

inline double getclampedpidwrist(double wristencval, double& pidout, RobotContainer* rc){
  pidout = rc->wrist_y_pid.Calculate(wristencval);
  if (pidout > 0.2)
    pidout = 0.2;
  if (pidout < -0.2)
    pidout = -0.2;

    return pidout;
};

/**
 * This function is called periodically during operator control.
 */

void Robot::TeleopPeriodic() 
{
  auto rc = RobotContainer::get();
  auto armdegrees = rc->m_arm_abs_encoder.GetDegrees();
  auto wristenc = rc->wrist_y_encoder.GetPosition();
  auto wy_deg = rc->wrist_y_degrees = t34::EncoderToDegree(44.4, rc->wrist_y_encoder.GetPosition());
  auto wr_deg = rc->wrist_rot_degrees = t34::EncoderToDegree(44.4, rc->wrist_rot_encoder.GetPosition());
  double rightstick_y = rc->m_driver_control->getRightStickYDB();
  double rightstick_x = rc->m_driver_control->getRightStickXDB();
  //double arm_setPoint = 0.0;
  double armpidout = 0.0;
  double wrpidout = 0.0;

  rc->m_front_cam.SetFPS(24);
  rc->m_front_cam.SetResolution(480, 270);

  // frc::SmartDashboard::PutNumber("Wrist y deg", wy_deg);
  // frc::SmartDashboard::PutNumber("Wrist rot deg", wr_deg);
  //frc::SmartDashboard::PutNumber("wrist Y Setpoint", rc->wrist_y_pid.GetSetpoint());
  frc::SmartDashboard::PutNumber("y axis navx", rc->m_drive->getYaw());
  // frc::SmartDashboard::PutNumber("wrist y encoder", rc->wrist_y_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Wrist rot encoder", rc->wrist_rot_encoder.GetPosition());

  //  WRIST DRIVE TO POS TOGGLE
  // if(rc->m_driver_control->GetYButtonReleased())
  // {
  //   rc->wristTog = !rc->wristTog;
  // }
  /*
 //   Wrist pitch control //
  if (rc->m_driver_control->GetPOV() == 180) {
    rc->m_wrist_y.Set(-1.0);
  }
  else
    rc->m_wrist_y.Set(0.0);

  if (rc->m_driver_control->GetPOV() == 0) {
    rc->m_wrist_y.Set(1.0);
  }
  else
    rc->m_wrist_y.Set(0.0);

  //    Wrist Rotation Control
  
  if (rc->m_driver_control->GetPOV() == 270) {
    rc->m_wrist_rot.Set(-1.0);
  }
  else
    rc->m_wrist_rot.Set(0.0);

  if (rc->m_driver_control->GetPOV() == 90) {
    rc->m_wrist_rot.Set(1.0);
  }
  else
    rc->m_wrist_rot.Set(0.0);

  }
  */
/*
   {
      auto current = rc->wrist_y_pid.GetSetpoint();

      if (current < -190.0) {
        rc->wrist_y_pid.SetSetpoint(-190.0);
        return;
      }
      rc->wrist_y_pid.SetSetpoint( current - 1.0);
}
else if (rc->m_driver_control->GetPOV() == 0) {
        auto current = rc->wrist_y_pid.GetSetpoint();

        if (current > 0.0) 
        {
           rc->wrist_y_pid.SetSetpoint(0.0);
           return;
        }

        rc->wrist_y_pid.SetSetpoint( current + 1.0);
}
rc->m_wrist_y.Set(ControlMode::PercentOutput, -rc->wrist_y_pid.Calculate(wy_deg));//-getclampedpidwrist(wy_deg, wrpidout, rc.get()));
*/

  //   rc->m_wrist_y.Set(ControlMode::Position, rc)
  // if (rc->wristTog == true && rc->m_arm_abs_encoder.GetDegrees() > 180.0 && rc->m_driver_control->GetPOV(360)) {
  //   rc->m_wrist_y.Set(ControlMode::Position, rc->wrist_y_pid.Calculate(rc->wrist_y_encoder.Get(), 90.0));
  // }
  // else if (rc->wristTog == true && rc->m_arm_abs_encoder.GetDegrees() < 180.0 && rc->m_driver_control->GetPOV(360)) {
  //   rc->m_wrist_y.Set(ControlMode::Position, rc->wrist_y_pid.Calculate(rc->wrist_y_encoder.Get(), -90.0));
  // }
  // else if (rc->wristTog == true && rc->m_driver_control->GetPOV(180)) {
  //   rc->m_wrist_y.Set(ControlMode::Position, rc->wrist_y_pid.Calculate(rc->wrist_y_encoder.Get(), 0));
  // }
  // else if (rc->wristTog == false)
  // {
  //   rc->m_wrist_y.Set(ControlMode::PercentOutput, 0.0);
  // }
  
  // frc::SmartDashboard::PutNumber("Wrist Rotation encoder val", rc->wrist_rot_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Wrist Pitch encoder val", rc->wrist_y_encoder.GetPosition());
  // frc::SmartDashboard::PutNumber("Arm ABS Encoder Degrees", armdegrees);
  // frc::SmartDashboard::PutNumber("Current arm SP", rc->arm_y_pid.GetSetpoint());

  //  Grip Solenoid, Button X
  if (rc->m_driver_control->GetXButtonReleased())
    rc->p_grip_solenoid->Toggle();

  //  ZERO YAW, Y
  if (rc->m_driver_control->GetYButtonReleased())
    rc->m_drive->zeroYaw();
  
  //  SheildWall & Toggle Drive Break, Button Select
  if (rc->m_driver_control->GetBackButtonReleased())
    rc->m_drive->sheildWall();

//  ARM PITCH CONTROL
// if (rc->m_driver_control->GetRightBumper())
// {
//   auto current = rc->arm_y_pid.GetSetpoint();

//   if (current > 333.0) 
//   {
//      rc->arm_y_pid.SetSetpoint(333.0);
//      return;
//   }

//   rc->arm_y_pid.SetSetpoint( current + t34::ARM_PITCH_VAL);
// }

// if (rc->m_driver_control->GetLeftBumper())
// {
//   auto current = rc->arm_y_pid.GetSetpoint();

//   if (current < 44.0) {
//     rc->arm_y_pid.SetSetpoint(44.0);
//      return;
//   }

//   rc->arm_y_pid.SetSetpoint( current - t34::ARM_PITCH_VAL);
// }


/*  
  auto pidout = rc->arm_y_pid.Calculate(armdegrees);
  if (pidout > 0.2)
    pidout = 0.2;
  if (pidout < -0.2)
    pidout = -0.2;
*/

//    Arm Control
if (rc->m_driver_control->GetLeftBumper()) {
  if (rc->m_limit_switch_back.Get()) {
        auto current = rc->arm_y_pid.GetSetpoint();

        if (current < 44.0) {
          rc->arm_y_pid.SetSetpoint(44.0);
           return;
        }

        rc->arm_y_pid.SetSetpoint( current - t34::ARM_PITCH_VAL);

       // rc->m_arm.Set(ControlMode::PercentOutput, getclampedpid(armdegrees, pidout, rc.get()));
  }

}
else if (rc->m_driver_control->GetRightBumper()) {
    if (rc->m_limit_switch_front.Get()) {
      if (rc->m_driver_control->GetRightBumper())
      {
        auto current = rc->arm_y_pid.GetSetpoint();

        if (current > 333.0) 
        {
           rc->arm_y_pid.SetSetpoint(333.0);
           return;
        }

        rc->arm_y_pid.SetSetpoint( current + t34::ARM_PITCH_VAL);
        
      }  
    }
}
rc->m_arm.Set(ControlMode::PercentOutput, getclampedpid(armdegrees, armpidout, rc.get()));

//else  
  //rc->m_arm.Set(ControlMode::PercentOutput, 0.0);
/*
if (rc->m_limit_switch_front.Get() == false && rc->m_arm.GetMotorOutputPercent() >= 0.1)
    rc->m_arm.Set(ControlMode::PercentOutput, 0.0);
else if (rc->m_limit_switch_back.Get() == false && rc->m_arm.GetMotorOutputPercent() <= -0.1)
    rc->m_arm.Set(ControlMode::PercentOutput, 0.0);
else
    {
      if (rc->m_driver_control->GetRightBumper())
      {
        auto current = rc->arm_y_pid.GetSetpoint();

        if (current > 333.0) 
        {
           rc->arm_y_pid.SetSetpoint(333.0);
           return;
        }

        rc->arm_y_pid.SetSetpoint( current + t34::ARM_PITCH_VAL);
      }

      if (rc->m_driver_control->GetLeftBumper())
      {
        auto current = rc->arm_y_pid.GetSetpoint();

        if (current < 44.0) {
          rc->arm_y_pid.SetSetpoint(44.0);
           return;
        }

        rc->arm_y_pid.SetSetpoint( current - t34::ARM_PITCH_VAL);
      }
    }
*/

//  WRIST ROTATION 
  // if(rc->arm_y_pid.GetSetpoint() > 180 && rc->wristTog == true)
  // {
  //   rc->m_wrist_rot.Set(ControlMode::PercentOutput, rc->wrist_rot_pid.Calculate(wr_deg, 0.0));
  // }
  // else if(rc->arm_y_pid.GetSetpoint() < 180 && rc->wristTog == true)
  // {
  //   rc->m_wrist_rot.Set(ControlMode::PercentOutput, rc->wrist_rot_pid.Calculate(wr_deg, 180.0));
  // }
  // else if (rc->wristTog == false)
  // {
  //   rc->m_wrist_rot.Set(ControlMode::PercentOutput, 0.0);
  // }
  

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
