#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/ArmSubsystem.h"

constexpr double ARM_ENCODER_OFFSET { 0.0 };
constexpr double ARM_GEAR_RATIO { 32.0 };
constexpr double ARM_ENCODER_UNITS { 2048.0 };
constexpr double ARM_ENCODER_DEGREES { 360.0 / ARM_ENCODER_UNITS };
constexpr double ARM_SCALAR { ARM_GEAR_RATIO * ARM_ENCODER_DEGREES };  

ArmSubsystem::ArmSubsystem()
    : m_motor(ID_ARM_PITCH_MOTOR) 
    , m_telemetry_on(false)
    , m_angle_increment(1.0)
    , m_sensor(m_motor.GetSensorCollection()) {
    
    m_motor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);
    m_motor.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToZero);

    SlotConfiguration arm_slot;
    m_motor.GetSlotConfigs(arm_slot);
    arm_slot.kP = 0.4;
    arm_slot.kI = 0.0;
    arm_slot.kD = 0.0;
    arm_slot.kF = 0.0;
    arm_slot.allowableClosedloopError = 2.0;
    arm_slot.closedLoopPeakOutput = 0.0;

    m_motor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    m_motor.ConfigureSlot(arm_slot, 0, 0);

}

void ArmSubsystem::Periodic() {
    if (m_telemetry_on) {
        frc::SmartDashboard::PutNumber("Arm Angle (DEG)", GetAngle());
        frc::SmartDashboard::PutNumber("Arm Angle (RAW)", GetRawAngle());
        // frc::SmartDashboard::PutNumber("Arm Output", GetOutputCurrent());
        // frc::SmartDashboard::PutNumber("Arm Stator", GetStatorCurrent());
        // frc::SmartDashboard::PutNumber("Arm Supply", GetSupplyCurrent());       
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
