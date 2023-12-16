// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>
#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/AnalogEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <AHRS.h>
#include <frc/DigitalInput.h>
#include <cameraserver/CameraServer.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SendableChooserBase.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h" 
#include "subsystems/ExampleSubsystem.h"
#include "utils/T34XboxController.h"
#include "commands/CMD_DefaultDrive.h"
#include "subsystems/ArmSubsystem.h"
#include "commands/CMD_DriveStraight.h"
#include "commands/CMD_ReturnHome.h"

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

    //cs::UsbCamera m_front_cam;

    AHRS navX;

    frc::SendableChooser<std::string> m_chooser;

    frc::DigitalInput m_limit_switch_back;
    frc::DigitalInput m_limit_switch_front;

    ArmSubsystem armSub;

    frc2::PIDController drive_pid;
    frc2::PIDController steer_pid;


    bool pneumatics_running;
    bool zeroed_steer;
    bool drive_braking;
    double wrist_y_degrees;
    double wrist_rot_degrees;
    double arm_degrees;
    double correction_val;
    
    std::shared_ptr<t34::T34XboxController> m_driver_control;
    std::shared_ptr<t34::SwerveDrive> m_drive; 

    //  COMMANDS
    t34::DefaultDriveCommand m_default_command;
    frc2::CommandPtr GetAutonomousCommand();
    static std::shared_ptr<frc2::Command> m_autonomousCommand;

 private:
    RobotContainer();
    // Replace with CommandPS4Controller or CommandJoystick if needed
    frc2::CommandXboxController m_driverController{
        t34::kDriverControllerPort};

    // The robot's subsystems are defined here...
    ExampleSubsystem m_subsystem;

    void ConfigureBindings();
};
