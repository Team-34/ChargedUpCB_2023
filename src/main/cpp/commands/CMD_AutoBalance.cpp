#include "RobotContainer.h"
#include "commands/CMD_AutoBalance.h"

namespace t34
{
    CMD_AutoBal::CMD_AutoBal()
    : PitchPID(0.0, 0.0, 1.0)
    , navX(new AHRS(frc::SPI::Port::kMXP))
    {}


    void CMD_AutoBal::Initialize()
    {
        PitchPID.SetTolerance(2.0,4.0);
        PitchPID.EnableContinuousInput(-180.0,180.0);
        PitchPID.SetSetpoint(0.0);
    }


    void CMD_AutoBal::Execute()
    {
        RobotContainer::get()->m_drive->manualdrive(PitchPID.Calculate(navX->GetPitch(), 0.0));
        
    }


    bool CMD_AutoBal::IsFinished()
    {
        if (PitchPID.AtSetpoint() == true)
        {
            RobotContainer::get()->m_drive->manualdrive(0.0);
        }
        return true;
    }
    

    void CMD_AutoBal::End()
    {

    }

}