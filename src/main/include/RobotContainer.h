// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <AHRS.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/ExampleSubsystem.h"
#include "utils/T34XboxController.h"
#include "commands/CMD_DefaultDrive.h"
#include "subsystems/ClawSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer 
{
 public:
  RobotContainer();

  static void initialize();

  static std::shared_ptr<t34::T34XboxController> m_driver_control;
  static std::shared_ptr<t34::SwerveDrive> m_drive; 

  static std::shared_ptr<TalonSRX> m_arm;
  static std::shared_ptr<rev::CANSparkMax> m_arm_ext;
  static std::shared_ptr<TalonSRX> m_wrist_y;
  static std::shared_ptr<TalonSRX> m_wrist_rot;

  static std::shared_ptr<frc::Encoder> arm_encoder;
  static std::shared_ptr<frc::Encoder> wrist_y_encoder;
  static std::shared_ptr<frc::Encoder> wrist_rot_encoder;


  static std::shared_ptr<frc::Solenoid> p_grip_solenoid;
  static std::shared_ptr<frc::Compressor> p_grip_compressor;

  static bool pneumatics_running;
  static bool drive_braking;

  static double wrist_degrees;
  static double arm_degrees;
  static double correction_val;

  static frc2::PIDController wrist_y_pid;
  static frc2::PIDController wrist_rot_pid;
  static frc2::PIDController arm_y_pid;
  static frc2::PIDController arm_ext_pid;

  //  COMMANDS
  static std::shared_ptr<t34::DefaultDriveCommand> m_default_command;


  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      t34::kDriverControllerPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;

  void ConfigureBindings();
};
