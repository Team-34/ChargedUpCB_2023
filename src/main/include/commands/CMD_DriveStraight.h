#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/MedianFilter.h>
#include "subsystems/SwerveDrive.h"

namespace t34
{
    class CMD_DriveStraightDistance : 
        public frc2::CommandHelper<frc2::CommandBase, CMD_DriveStraightDistance>
    {
    public:
        
        CMD_DriveStraightDistance(float distance,  double speed, double rotationSpeed);
        
        virtual void Initialize();

        virtual void Execute();

        virtual bool IsFinished();

        virtual void End(); 

        virtual wpi::SmallSet<frc2::Subsystem*, 4> GetRequirements() const;

    private:
        double m_setpoint;
        double m_speed;
        double m_original_speed;
        double m_kP = 0.000001;
        double m_distance;
        double m_encoderDriven;
        double m_initialEncoder;
        double m_encoderBetweenLoop;
        double m_rotationSpeed;
    };
}