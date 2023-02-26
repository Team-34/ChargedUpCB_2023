// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>


#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

// std::shared_ptr<TalonSRX> RobotContainer::m_arm{ nullptr };
// std::shared_ptr<TalonSRX> RobotContainer::m_wrist_y{ nullptr };
// std::shared_ptr<TalonSRX> RobotContainer::m_wrist_rot{ nullptr };
// std::shared_ptr<t34::T34XboxController> RobotContainer::m_driver_control{ nullptr };
// std::shared_ptr<t34::DefaultDriveCommand> RobotContainer::m_default_command{ nullptr };
// std::shared_ptr<frc::Encoder> RobotContainer::arm_encoder{ nullptr };
// std::shared_ptr<rev::CANSparkMax> RobotContainer::m_arm_ext{ nullptr };
// std::shared_ptr<frc::Encoder> RobotContainer::wrist_y_encoder{ nullptr };
// std::shared_ptr<frc::Encoder> RobotContainer::wrist_rot_encoder{ nullptr };
// std::shared_ptr<t34::SwerveDrive> RobotContainer::m_drive{ nullptr };
// std::shared_ptr<frc::Solenoid> RobotContainer::p_grip_solenoid{ nullptr };
// std::shared_ptr<frc::Compressor> RobotContainer::p_grip_compressor{ nullptr };




std::shared_ptr<RobotContainer> g_container{ nullptr };

std::shared_ptr<RobotContainer> RobotContainer::get() {
    if (!g_container)
        g_container.reset(new RobotContainer());

    return g_container;
}

RobotContainer::RobotContainer() 
    : m_driver_control(new t34::T34XboxController(ID_DRIVE_CONTROLLER)) 
    , m_drive(new t34::SwerveDrive())
    , m_default_command(m_drive, m_driver_control) 
    , wrist_y_pid(frc::PIDController(0.05, 0.0, .0))
    , wrist_rot_pid(frc::PIDController(0.05, 0.0, .0))
    , arm_y_pid(frc::PIDController(0.05, 0.0, .0))
    , arm_ext_pid(frc::PIDController(0.05, 0.0, .0)) {

    m_arm.reset(new TalonSRX(ID_ARM_PITCH_MOTOR));
    m_arm_ext.reset(new rev::CANSparkMax(ID_ARM_EXT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
    m_wrist_y.reset(new TalonSRX(ID_WRIST_Y_MOTOR));
    m_wrist_rot.reset(new TalonSRX(ID_WRIST_ROT_MOTOR));

    //RobotContainer::arm_ext_encoder.reset(new rev::SparkMaxRelativeEncoder(m_arm_ext)));
    arm_encoder.reset(new frc::Encoder(0,1));
    wrist_y_encoder.reset(new frc::Encoder(2,3));
    wrist_rot_encoder.reset(new frc::Encoder(4,5));
    wrist_y_encoder->SetDistancePerPulse(360.0 / 44.4);


    p_grip_solenoid.reset(new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, 0));
    p_grip_compressor.reset(new frc::Compressor(1, frc::PneumaticsModuleType::CTREPCM));
    m_driver_control->setAllAxisDeadband(0.2);
    wrist_y_pid.EnableContinuousInput(-180.0, 180.0);
    wrist_rot_pid.EnableContinuousInput(-180.0, 180.0);

    pneumatics_running = false;
    drive_braking = false;
    wrist_degrees = 0.0 ;
    arm_degrees = 0.0 ;
    correction_val = 0.0 ;

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
