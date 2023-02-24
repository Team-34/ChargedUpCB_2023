// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>


#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

std::shared_ptr<TalonSRX> RobotContainer::m_arm{ nullptr };
std::shared_ptr<TalonSRX> RobotContainer::m_wrist_y{ nullptr };
std::shared_ptr<TalonSRX> RobotContainer::m_wrist_rot{ nullptr };
std::shared_ptr<t34::T34XboxController> RobotContainer::m_driver_control{ nullptr };
std::shared_ptr<t34::DefaultDriveCommand> RobotContainer::m_default_command{ nullptr };
std::shared_ptr<frc::Encoder> RobotContainer::arm_encoder{ nullptr };
std::shared_ptr<rev::CANSparkMax> RobotContainer::m_arm_ext{ nullptr };
std::shared_ptr<frc::Encoder> RobotContainer::wrist_y_encoder{ nullptr };
std::shared_ptr<frc::Encoder> RobotContainer::wrist_rot_encoder{ nullptr };
std::shared_ptr<t34::SwerveDrive> RobotContainer::m_drive{ nullptr };
std::shared_ptr<frc::Solenoid> RobotContainer::p_grip_solenoid{ nullptr };
std::shared_ptr<frc::Compressor> RobotContainer::p_grip_compressor{ nullptr };

frc2::PIDController RobotContainer::wrist_y_pid{0.05, 0.0, .0};
frc2::PIDController RobotContainer::wrist_rot_pid{0.05, 0.0, .0};
frc2::PIDController RobotContainer::arm_y_pid{0.05, 0.0, .0};
frc2::PIDController RobotContainer::arm_ext_pid{0.05, 0.0, .0};

bool RobotContainer::pneumatics_running{false};
bool RobotContainer::drive_braking{false};

double RobotContainer::wrist_degrees{ 0.0 };
double RobotContainer::arm_degrees{ 0.0 };
double RobotContainer::correction_val{ 0.0 };

void RobotContainer::initialize() 
{
    RobotContainer::m_driver_control.reset(new t34::T34XboxController(ID_DRIVE_CONTROLLER)); 
    RobotContainer::m_drive.reset(new t34::SwerveDrive());
    RobotContainer::m_default_command.reset(new t34::DefaultDriveCommand(m_drive, m_driver_control)); 

    RobotContainer::m_arm.reset(new TalonSRX(ID_ARM_MOTOR));
    RobotContainer::m_arm_ext.reset(new rev::CANSparkMax(ID_ARM_EXT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
    RobotContainer::m_wrist_y.reset(new TalonSRX(ID_WRIST_Y_MOTOR));
    RobotContainer::m_wrist_rot.reset(new TalonSRX(ID_WRIST_ROT_MOTOR));

    RobotContainer::arm_encoder.reset(new frc::Encoder(0,1));
    RobotContainer::wrist_y_encoder.reset(new frc::Encoder(2,3));
    RobotContainer::wrist_rot_encoder.reset(new frc::Encoder(4,5));
    RobotContainer::wrist_y_encoder->SetDistancePerPulse(360.0 / 44.4);


    RobotContainer::p_grip_solenoid.reset(new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, 0));
    RobotContainer::p_grip_compressor.reset(new frc::Compressor(1, frc::PneumaticsModuleType::CTREPCM));
    RobotContainer::m_driver_control->setAllAxisDeadband(0.2);
    RobotContainer::wrist_y_pid.EnableContinuousInput(-180.0, 180.0);
    RobotContainer::wrist_rot_pid.EnableContinuousInput(-180.0, 180.0);
}

RobotContainer::RobotContainer() 
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
