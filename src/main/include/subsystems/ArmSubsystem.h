#pragma once

#include "Constants.h"
#include "ArmAbsEncoder.h"
#include "utils/T34XboxController.h"

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <frc/controller/PIDController.h>
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>

class ArmSubsystem : frc2::SubsystemBase {
public:
    ArmSubsystem();

    rev::CANSparkMax m_wrist_y;
    rev::CANSparkMax m_wrist_rot;
    TalonSRX m_arm_ext;
    TalonFX m_arm;

    ArmAbsEncoder m_arm_abs_encoder;
    frc::Encoder arm_ext_encoder;
    rev::SparkMaxRelativeEncoder wrist_y_encoder;
    rev::SparkMaxRelativeEncoder wrist_rot_encoder;

    frc2::PIDController wrist_y_pid;
    frc2::PIDController wrist_rot_pid;
    frc2::PIDController arm_y_pid;
    frc2::PIDController arm_ext_pid;

    double ext_sp;
    double wrist_y_degrees;
    double wrist_rot_degrees;
    double arm_degrees;
    double w_degrees = wrist_y_encoder.GetPosition();
    double w_rot_degrees = wrist_rot_encoder.GetPosition();

    SlotConfiguration arm_slot;

    std::shared_ptr<frc::Solenoid> p_grip_solenoid;
    std::shared_ptr<frc::Compressor> p_grip_compressor;

    void Periodic() override;

    void SetAngleIncrement(double value); 
    inline void Increment() { SetRawAngle(m_setpoint + m_angle_increment); }
    inline void Decrement() { SetRawAngle(m_setpoint - m_angle_increment); }
    void operator++() { SetRawAngle(m_setpoint + m_angle_increment); }
    void operator--() { SetRawAngle(m_setpoint - m_angle_increment); }

    double GetAngle();
    double GetRawAngle(bool with_offset = true);
    void Arm_Pitch_Cntrl();

    void SetAngle(double degrees);
    void SetRawAngle(double encoder_units, bool apply_offset = true);
    
    void ArmExtZero();

    inline void TelemetryOn(bool on = true) { m_telemetry_on = on; }


private:
    //TalonFX m_motor;
    bool m_telemetry_on;
    double m_angle_increment;
    double m_setpoint;
    TalonFXSensorCollection &m_sensor;
};
