#include "RobotContainer.h"
#include "commands/CMD_AutoBalance.h"

namespace t34
{
    CMD_AutoBal::CMD_AutoBal()
    : PitchPID(0.0, 0.0, 1.0)
    {}


    void CMD_AutoBal::Initialize()
    {
        auto rc = RobotContainer::get();
        PitchPID.SetTolerance(1.0,0.0);
        PitchPID.EnableContinuousInput(-180.0,180.0);
        PitchPID.SetSetpoint(0.0);
        rc->m_drive->zeroDrive();
        gyro = rc->m_drive->m_gyro.get();
    }


    void CMD_AutoBal::Execute()
    {
        frc::SmartDashboard::PutNumber("Gyro pitch", gyro->GetRoll());
        RobotContainer::get()->m_drive->drive(0.0, std::clamp(PitchPID.Calculate(gyro->GetRoll(), 0.0),-0.3, 0.3), 0.0);
    }


    bool CMD_AutoBal::IsFinished()
    {
        if (PitchPID.AtSetpoint() == true)
        {
            RobotContainer::get()->m_drive->drive(0.0, 0.0, 0.0);
        }
        return true;
    }
    

    void CMD_AutoBal::End()
    {

    }

}