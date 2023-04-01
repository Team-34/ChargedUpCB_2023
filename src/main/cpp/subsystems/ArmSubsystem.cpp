#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotContainer.h"
#include "subsystems/ArmSubsystem.h"

constexpr double ARM_ENCODER_OFFSET { 0.0 };
constexpr double ARM_GEAR_RATIO { 32.0 };
constexpr double ARM_ENCODER_UNITS { 2048.0 };
constexpr double ARM_ENCODER_DEGREES { 360.0 / ARM_ENCODER_UNITS };
constexpr double ARM_SCALAR { ARM_GEAR_RATIO * ARM_ENCODER_DEGREES };  
auto rc = RobotContainer::get();

ArmSubsystem::ArmSubsystem()
    : m_arm_ext(TalonSRX(ID_ARM_EXT_MOTOR))
    , m_arm(ID_ARM_PITCH_MOTOR)
    , arm_ext_encoder(0, 1)
    , m_telemetry_on(false)
    , m_angle_increment(1.0)
    , m_sensor(m_arm.GetSensorCollection())
    , m_arm_abs_encoder(ID_ARM_ABS_ENCODER)
    , arm_y_pid(frc2::PIDController(0.01, 0.0, 0.0001))
    , arm_ext_pid(frc2::PIDController(0.05, 0.0, 0.0))
    , wrist_y_pid(frc2::PIDController(0.05, 0.0, 0.0))
    , wrist_rot_pid(frc2::PIDController(0.05, 0.0, 0.0))
    , m_wrist_y(rev::CANSparkMax(ID_WRIST_Y_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless))
    , m_wrist_rot(rev::CANSparkMax(ID_WRIST_ROT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless))
    , wrist_y_encoder(m_wrist_y.GetEncoder())
    , wrist_rot_encoder(m_wrist_rot.GetEncoder()) 
    {
    
    m_arm.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);
    m_arm.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToZero);

    SlotConfiguration arm_slot;
    m_arm.GetSlotConfigs(arm_slot);
    arm_slot.kP = 0.4;
    arm_slot.kI = 0.0;
    arm_slot.kD = 0.0;
    arm_slot.kF = 0.0;
    arm_slot.allowableClosedloopError = 2.0;
    arm_slot.closedLoopPeakOutput = 0.0;

    m_arm.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    m_arm.ConfigureSlot(arm_slot, 0, 0);

    arm_y_pid.EnableContinuousInput(0.0, 360.0);
    wrist_y_pid.EnableContinuousInput(-180.0, 180.0);
    wrist_rot_pid.EnableContinuousInput(-180.0, 180.0);

    double wrist_y_degrees = 0.0;
    double wrist_rot_degrees = 0.0;
    double arm_degrees = 0.0;

    p_grip_solenoid.reset(new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, 0));
    p_grip_compressor.reset(new frc::Compressor(0, frc::PneumaticsModuleType::CTREPCM));

    m_wrist_y.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_wrist_rot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_arm.SetNeutralMode(NeutralMode::Brake);
    m_arm_ext.SetNeutralMode(Coast);


    m_arm_abs_encoder.Reset();
    m_arm_abs_encoder.SetDistancePerRotation(2048.0 * 32.0);


    wrist_y_pid.SetTolerance(2, 3);
    wrist_rot_pid.SetTolerance(2, 3);
    wrist_y_encoder.SetPositionConversionFactor(1.0);
    wrist_rot_encoder.SetPositionConversionFactor(1.0);
    wrist_y_encoder.SetPosition(-24.0);
    wrist_rot_encoder.SetPosition(0.0);


    m_wrist_y.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_wrist_y.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    m_wrist_y.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0.0);
    m_wrist_y.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -60.0);
    m_arm_ext.ConfigForwardSoftLimitEnable(false);
    m_arm_ext.ConfigReverseSoftLimitEnable(false);
    m_arm_ext.ConfigForwardSoftLimitThreshold(5484.0);
    m_arm_ext.ConfigReverseSoftLimitThreshold(0.0);

}

void ArmSubsystem::Periodic() {
    if (m_telemetry_on) {
        frc::SmartDashboard::PutNumber("Arm Angle (DEG)", GetAngle());
        frc::SmartDashboard::PutNumber("Arm Angle (RAW)", GetRawAngle());    
    }
}

void ArmSubsystem::SetAngleIncrement(double value) { 
    m_angle_increment = value / ARM_SCALAR; 
}

double ArmSubsystem::GetAngle() {
    return m_sensor.GetIntegratedSensorAbsolutePosition() * ARM_SCALAR - ARM_ENCODER_OFFSET;
    //return m_motor.GetSelectedSensorPosition() * ARM_SCALAR - ARM_ENCODER_OFFSET;
}

double ArmSubsystem::GetRawAngle(bool with_offset) {
    return (m_sensor.GetIntegratedSensorAbsolutePosition() + (with_offset ? ARM_ENCODER_OFFSET : 0.0));
    //return (m_motor.GetSelectedSensorPosition() + (with_offset ? ARM_ENCODER_OFFSET : 0.0));
}

void ArmSubsystem::SetAngle(double degrees) {
    if (degrees < 0.0 || degrees > 360.0)
        return;
    
    m_setpoint = degrees / ARM_SCALAR + ARM_ENCODER_OFFSET;
//    m_motor.Set(ControlMode::Position, m_setpoint);
}

void ArmSubsystem::SetRawAngle(double encoder_units, bool apply_offset) {
    if (encoder_units < 0.0 || encoder_units > 4096.0)
        return;

    m_setpoint = encoder_units + (apply_offset ? ARM_ENCODER_OFFSET : 0.0);
//    m_motor.Set(ControlMode::Position, m_setpoint);
}

void ArmSubsystem::ArmExtZero() {
    if (m_arm_ext.IsRevLimitSwitchClosed()) {
        arm_ext_encoder.Reset();
    }
}

void ArmSubsystem::Arm_Pitch_Cntrl()
{
    if (rc->m_driver_control->GetRightBumper()) {
    if (rc->m_limit_switch_back.Get()) {
          auto current = rc->armSub.arm_y_pid.GetSetpoint();

          if (current < 44.0) {
            rc->armSub.arm_y_pid.SetSetpoint(44.0);
          }

          else {
            rc->armSub.arm_y_pid.SetSetpoint( current - t34::ARM_PITCH_VAL);
          }
    }

  }
  else if (rc->m_driver_control->GetLeftBumper()) {
      if (rc->m_limit_switch_front.Get()) {
          auto current = rc->armSub.arm_y_pid.GetSetpoint();

          if (current > 333.0) 
          {
             rc->armSub.arm_y_pid.SetSetpoint(333.0);
          }

          else {
            rc->armSub.arm_y_pid.SetSetpoint( current + t34::ARM_PITCH_VAL);
          }

      }
  }
} 
