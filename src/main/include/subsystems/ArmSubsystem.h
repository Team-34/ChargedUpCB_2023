#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

class ArmSubsystem : frc2::SubsystemBase {
public:
    ArmSubsystem();

    void Periodic() override;

    void SetAngleIncrement(double value); 
    inline void Increment() { SetRawAngle(m_setpoint + m_angle_increment); }
    inline void Decrement() { SetRawAngle(m_setpoint - m_angle_increment); }
    void operator++() { SetRawAngle(m_setpoint + m_angle_increment); }
    void operator--() { SetRawAngle(m_setpoint - m_angle_increment); }

    double GetAngle();
    double GetRawAngle(bool with_offset = true);

    void SetAngle(double degrees);
    void SetRawAngle(double encoder_units, bool apply_offset = true);

    inline void TelemetryOn(bool on = true) { m_telemetry_on = on; }


private:
    TalonFX m_motor;
    bool m_telemetry_on;
    double m_angle_increment;
    double m_setpoint;
    TalonFXSensorCollection &m_sensor;
};
