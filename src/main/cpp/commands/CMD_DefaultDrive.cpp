#include "commands/CMD_DefaultDrive.h"
#include <iostream>

namespace t34 
{
    DefaultDriveCommand::DefaultDriveCommand(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<T34XboxController> controller)
        : m_drive(drive)
        , m_driver_control(controller) {
        
        AddRequirements(drive.get());

    }

    void DefaultDriveCommand::Initialize() {
    }

    void DefaultDriveCommand::Execute() {
        //std::cout << "Encoder Driven: " << m_drive->getOdometer() << std::endl;
       //double x = m_driver_control->getLeftStickXDB(); 
       //double y =           m_driver_control->getLeftStickYDB(); 
       //double z =           m_driver_control->getTriggersCoercedDB();
               m_drive->drive(m_driver_control->getLeftStickXDB(), 
                  m_driver_control->getLeftStickYDB(), 
                  m_driver_control->getTriggersCoercedDB());
      
    }



}