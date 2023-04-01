#include "commands/CMD_DriveStraight.h"
#include <iostream>
#include "Constants.h"
#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>

namespace t34
{
    auto rc = RobotContainer::get();

    CMD_DriveStraightDistance::CMD_DriveStraightDistance(float distance, double speed, double rotationSpeed){
        m_distance = distance;
        m_speed = speed;
        m_rotationSpeed = rotationSpeed;
    }

    void CMD_DriveStraightDistance::Initialize() 
    {   
        std::cout << "DS Init Called (" << m_distance << ")\n";

        m_encoderDriven = 0.0;
        rc->m_drive->resetOdometer();
        m_encoderBetweenLoop = rc->m_drive->getOdometer();
    }

    void CMD_DriveStraightDistance::Execute() 
    {
        auto rc = RobotContainer::get();
        auto od = rc->m_drive->getOdometer();
        auto d_pid = std::clamp(rc->drive_pid.Calculate(rc->m_drive->getOdometer(), m_distance), -0.2, 0.2);
        // frc::SmartDashboard::PutNumber("OD", od);
        // frc::SmartDashboard::PutNumber("OD PRE LOOP", m_encoderBetweenLoop);
        // m_encoderBetweenLoop = rc->m_drive->getOdometer() - m_encoderBetweenLoop;
        // frc::SmartDashboard::PutNumber("OD POST LOOP", m_encoderBetweenLoop);
        // m_encoderDriven += m_encoderBetweenLoop;
        // frc::SmartDashboard::PutNumber("OD DRIVEN", m_encoderDriven);
        //double speed = (m_distance - m_encoderDriven) * m_kP;
        //double rotation = (rc->m_drive->getYaw()) * m_rotationSpeed;
        rc->m_drive->drive(0.0 /*m_speed*/, d_pid, 0.0);
        frc::SmartDashboard::PutNumber("pid output", d_pid);
    }

    bool CMD_DriveStraightDistance::IsFinished()
    {
        auto rc = RobotContainer::get();

        if (rc->m_drive->getOdometer() /*m_encoderDriven*/ >= m_distance){
            rc->m_drive->m_la->drive->Set(ControlMode::PercentOutput, 0);
            rc->m_drive->m_lf->drive->Set(ControlMode::PercentOutput, 0);
            rc->m_drive->m_ra->drive->Set(ControlMode::PercentOutput, 0);
            rc->m_drive->m_rf->drive->Set(ControlMode::PercentOutput, 0);
            std::cout << "Finished driving to distance" << std::endl;
            return true;
        }
        return false;
    }

    void CMD_DriveStraightDistance::End()
    {
//        std::cout << "DriveToDistanceCommand::End()\n";
    }

    wpi::SmallSet<frc2::Subsystem*, 4> CMD_DriveStraightDistance::GetRequirements() const
    {
        wpi::SmallSet<frc2::Subsystem*, 4> set;
        set.insert(rc->m_drive.get());
        return set;
    }

}