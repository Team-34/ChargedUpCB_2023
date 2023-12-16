// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

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
    , drive_pid(frc::PIDController(0.1, 0.0, 0.0))
    , steer_pid(frc::PIDController(0.1, 0.0, 0.0))
    , m_limit_switch_back(5)
    , m_limit_switch_front(4)
  // , m_front_cam("FRONT CAM", 0)
    , navX(frc::SPI::Port::kMXP)
    {


    armSub.TelemetryOn();
    m_driver_control->setAllAxisDeadband(0.2);
    

    drive_braking = false;

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
