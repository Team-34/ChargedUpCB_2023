#include "subsystems/ClawSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "RobotContainer.h"
#include "ctre/Phoenix.h"

namespace t34
{
  double EncoderToDegree(double full_rot_encoder, double current_rot_encoder)
    {
      double rot_encoder = current_rot_encoder / full_rot_encoder;
      return rot_encoder * 360.0;
    }
  
  
  double CorrectionValue(double arm, double wrist)
    {
      return wrist - arm;
    }
  
  
  void WristStabilize(double wrist_degrees, double arm_degrees) 
    {
      frc::SmartDashboard::PutNumber("W Current Position", wrist_degrees);
      RobotContainer::m_wrist_y->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, RobotContainer::wrist_pid.Calculate(wrist_degrees) * 0.2);
    }


}