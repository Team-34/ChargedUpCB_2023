#pragma once

#include <memory>
#include <iostream>

#include <AHRS.h>

#include <frc/AnalogInput.h>

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/voltage.h>
#include <units/base.h>
#include <frc/controller/PIDController.h>


namespace t34 {

    enum class DriveMode {
        RobotCentric,
        FieldOriented
    };

    class SwerveModule {
    public:
        SwerveModule(std::string name, const int drive_id, const int steer_id, int invert, const int encoder_id, double offset) 
            : drive(TalonFX(drive_id))
            , steer(TalonFX(steer_id))
            , encoder(encoder_id) 
            , zero_offset(offset)
            , invert_value(invert)  
            , module_name(name)
             {

            SlotConfiguration slot;
            steer.GetSlotConfigs(slot);
            slot.kP = 0.4;
            slot.kI = 0.0;
            slot.kD = 0.0;
            slot.kF = 0.0;
            //slot.integralZone = 180.0;
            slot.allowableClosedloopError = 2.0;
            //slot.maxIntegralAccumulator = 360.0;
            slot.closedLoopPeakOutput = 1.0;

            // Configure Steering Encoder
            encoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
            encoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
            encoder.ConfigMagnetOffset(offset);
            encoder.SetPositionToAbsolute();

            // Configure Steer Motor
            steer.ConfigRemoteFeedbackFilter(encoder_id, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
            steer.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0);
            steer.ConfigureSlot(slot, 0, 0);
            steer.SetNeutralMode(NeutralMode::Coast);
            steer.ConfigFeedbackNotContinuous(false);


            // Configure Drive Motor
            drive.ConfigFactoryDefault();
            setDriveBrake(true);
        }

        inline void setDriveBrake(bool on = true) { drive.SetNeutralMode(on ? NeutralMode::Brake : NeutralMode::Coast); }

        void zeroSteer() { 
            steer.Set(ControlMode::Position, 0.0);
        }


        void setSteerPosition(const double& position, double offset = 0.0) 
        {
            double current_position = encoder.GetAbsolutePosition();
            double set_point = (position + 180.0) / 360.0 * FULL_UNITS;
            frc::PIDController steer_pid( 0.5, 0.0, 0.0 );
            
            double delta = fmod(set_point - current_position, FULL_UNITS);

            //Calculating Shortest Distance
             if(fabs(delta) > 1024.0) {
                 delta -= copysign(2048.0, delta);
                 invert_value = -1.0;
             }
             else 
                 invert_value = 1.0;
        
            steer.Set(ControlMode::Position, steer_pid.Calculate(current_position + delta +  (offset < 2048.0 ? -offset : offset)));
        }

        TalonFX drive;
        TalonFX steer;
        CANCoder encoder;
        
        double zero_offset;
        double invert_value; 

        std::string module_name;
    };

    class SwerveDrive : public frc2::SubsystemBase {
    public:
        SwerveDrive();
        ~SwerveDrive();

        virtual void Periodic();

        // Gyro
        inline double getHeading() {return m_gyro.GetFusedHeading();}
        inline double getYaw() { return m_gyro.GetYaw(); }
        void zeroYaw();
        
        // Drive Mode
        void setDriveMode(DriveMode dm) { m_mode = dm; }
        void toggleDriveMode();
        
        // Drive Brakes
        void setDriveBrake(bool on = true);
        void toggleDriveBrake();
        
        // Drive Controls
        void drive(double x, double y, double r);
        void sheildWall();

        inline void setSpeed(double speed) { m_speed = speed; }
        inline double getSpeed() { return m_speed; }
        void toggleSpeed();

        void manualdrive(double percentoutput);
        
        // Odometer
        void resetOdometer();
        double getOdometer();

        // Smart Dashboard
        void putTelemetry();

        inline bool isSteeringZeroed() { return m_is_steering_zeroed; }
        void zeroSteering();

        void InitDefaultDrive();

        SwerveModule m_lf;
        SwerveModule m_la;
        SwerveModule m_rf;
        SwerveModule m_ra;

    private:
        AHRS m_gyro;
        double m_heading_offset;
        DriveMode m_mode;
        /*
        frc::SlewRateLimiter<double> la_sr{0.5};
        frc::SlewRateLimiter<double> ra_sr{0.5};
        frc::SlewRateLimiter<double> lf_sr{0.5};
        frc::SlewRateLimiter<double> rf_sr{0.5};
        */
        bool m_drive_brake_on;

        double m_db;
        double m_speed;

        bool m_is_steering_zeroed;
    };
}

