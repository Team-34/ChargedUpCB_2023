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
#include <frc/AnalogEncoder.h>
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
    static std::shared_ptr<RobotContainer> get();

    frc::AnalogInput m_arm_encoder;

    std::shared_ptr<t34::T34XboxController> m_driver_control;
    std::shared_ptr<t34::SwerveDrive> m_drive; 
    std::shared_ptr<TalonFX> m_arm;
    rev::CANSparkMax m_arm_ext;
    std::shared_ptr<TalonSRX> m_wrist_y;
    std::shared_ptr<TalonSRX> m_wrist_rot;
    //std::shared_ptr<frc::AnalogEncoder> arm_encoder;
    std::shared_ptr<frc::Encoder> wrist_y_encoder;
    std::shared_ptr<frc::Encoder> wrist_rot_encoder;
    std::shared_ptr<rev::SparkMaxRelativeEncoder> arm_ext_encoder;
    std::shared_ptr<frc::Solenoid> p_grip_solenoid;
    std::shared_ptr<frc::Compressor> p_grip_compressor;
    bool pneumatics_running;
    bool drive_braking;
    double wrist_degrees;
    double arm_degrees;
    double correction_val;
    frc2::PIDController wrist_y_pid;
    frc2::PIDController wrist_rot_pid;
    frc2::PIDController arm_y_pid;
    frc2::PIDController arm_ext_pid;

    //  COMMANDS
    t34::DefaultDriveCommand m_default_command;


    frc2::CommandPtr GetAutonomousCommand();

 private:
    RobotContainer();
    // Replace with CommandPS4Controller or CommandJoystick if needed
    frc2::CommandXboxController m_driverController{
        t34::kDriverControllerPort};

    // The robot's subsystems are defined here...
    ExampleSubsystem m_subsystem;

    void ConfigureBindings();
};
