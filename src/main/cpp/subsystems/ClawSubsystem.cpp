#include "subsystems/ClawSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "RobotContainer.h"
#include "ctre/Phoenix.h"

namespace t34
{
constexpr double jpencoder{ 360.0 / 44.4 };
  double EncoderToDegree(double full_rot_encoder, double current_rot_encoder)
    {
      //double rot_encoder = current_rot_encoder / full_rot_encoder;
      //return rot_encoder * 360.0;
      return current_rot_encoder * jpencoder;
    }
  
  
  double CorrectionValue(double arm, double wrist)
    {
      return 90 - arm;
    }
  
  
  void WristStabilize(double wrist_degrees, double arm_degrees) 
    {
      auto rc = RobotContainer::get();
      frc::SmartDashboard::PutNumber("W Current Position", wrist_degrees);
      rc->m_wrist_y.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rc->wrist_y_pid.Calculate(wrist_degrees) * 0.2);
    }


}