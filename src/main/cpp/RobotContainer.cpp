// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "Constants.h"
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
    , wrist_y_pid(frc2::PIDController(0.05, 0.0, 0.0))
    , wrist_rot_pid(frc2::PIDController(0.05, 0.0, 0.0))
    , arm_y_pid(frc2::PIDController(0.01, 0.0, 0.0001))
    , arm_ext_pid(frc2::PIDController(0.05, 0.0, 0.0)) 
    , drive_pid(frc::PIDController(0.1, 0.0, 0.0))
    , m_arm_ext(TalonSRX(ID_ARM_EXT_MOTOR)) 
    , m_wrist_y(rev::CANSparkMax(ID_WRIST_Y_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless))
    , m_wrist_rot(rev::CANSparkMax(ID_WRIST_ROT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless))
    , wrist_y_encoder(m_wrist_y.GetEncoder())
    , wrist_rot_encoder(m_wrist_rot.GetEncoder())
    , arm_ext_encoder(0, 1)
    , m_arm_abs_encoder(ID_ARM_ABS_ENCODER)
    , m_limit_switch_back(5)
    , m_limit_switch_front(4)
    , m_arm(ID_ARM_PITCH_MOTOR)
    , m_front_cam("FRONT CAM", 0)
    , navX(frc::SPI::Port::kMXP)
    //, table(nt::NetworkTableInstance::GetDefault().GetTable("limelight"))
    //, targetOffsetAngle_Horizontal(table->GetNumber("tx",0.0))
    //, targetOffsetAngle_Vertical(table->GetNumber("ty",0.0))
    //, targetArea(table->GetNumber("ta",0.0))
    //, targetSkew(table->GetNumber("ts",0.0))
    //, start_sec() 
    {

    m_arm.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);

    armSub.TelemetryOn();

    SlotConfiguration arm_slot;
    m_arm.GetSlotConfigs(arm_slot);
    arm_slot.kP = 0.4;
    arm_slot.kI = 0.0;
    arm_slot.kD = 0.0;
    arm_slot.kF = 0.0;
    arm_slot.allowableClosedloopError = 2.0;
    arm_slot.closedLoopPeakOutput = 1.0;


    p_grip_solenoid.reset(new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, 0));
    p_grip_compressor.reset(new frc::Compressor(0, frc::PneumaticsModuleType::CTREPCM));
    m_driver_control->setAllAxisDeadband(0.2);
    wrist_y_pid.EnableContinuousInput(-180.0, 180.0);
    wrist_rot_pid.EnableContinuousInput(-180.0, 180.0);
    arm_y_pid.EnableContinuousInput(0.0, 360.0);
    

    drive_braking = false;
    wrist_y_degrees = 0.0 ;
    wrist_rot_degrees = 0.0 ;
    arm_degrees = 0.0 ;
    correction_val = 0.0 ;

    //std::shared_ptr<frc2::Command> m_autonomousCommand{ nullptr };

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


/*double RobotContainer::T34AnalogInput::GetAbsoluteArmPitch() 
{
  double NewVoltage = GetVoltage();

  if (GetVoltage() >= 0.023 && GetVoltage() < 2.523)
  {
    
  }

  return NewVoltage;
};*/

//double RobotContainer::T34AnalogInput::GetArmPitchDeg(double encoder_val){  return (encoder_val-1.669) * t34::ARM_PITCH_DEGREE;  };