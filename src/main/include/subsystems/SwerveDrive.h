#pragma once

#include <memory>

#include <AHRS.h>

#include <frc/AnalogInput.h>

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/sensors/CANCoder.h>


namespace t34 {

    enum class DriveMode {
        RobotCentric,
        FieldOriented
    };



    class SwerveModule {
    public:
        SwerveModule(std::string name, const int drive_id, const int steer_id, int invert, const int encoder_id) 
            : drive(new WPI_TalonFX(drive_id))
            , steer(new WPI_TalonFX(steer_id))
            , encoder(new ctre::phoenix::sensors::CANCoder(encoder_id, "CANcoder"))
            , offset(fabs(steer->GetSelectedSensorPosition())) 
            , module_name(name)
            , invert_value(invert) 
            {

            // Configure Drive Motor
            drive->ConfigFactoryDefault();
            drive->SetNeutralMode(NeutralMode::Coast);
            setDriveBrake(true);
            // ??? SETTINGS FOR DRIVE ???

            // Configure Steer Motor
            steer->ConfigFactoryDefault();            
            steer->SetNeutralMode(NeutralMode::Coast);
            steer->ConfigFeedbackNotContinuous(false);
            steer->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
            steer->ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180);
            steer->ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition);
            steer->Config_kP(0, 0.4, 0.0);
            steer->Config_kD(0, 0.0, 0.0);
            
            // Configure Encoder 
            //offset = encoder->GetValue() * ABS_TO_IS;

        }

        inline void setDriveBrake(bool on = true) { drive->SetNeutralMode(on ? NeutralMode::Brake : NeutralMode::Coast); }

        inline void zeroSteer() { steer->Set(ControlMode::Position, -offset); }


        void setSteerPosition(std::shared_ptr<SwerveModule> sm, const double& position, double offset = 0.0) 
        {
            double current_position = sm->steer->GetSelectedSensorPosition();
            double set_point = (position + 180) / 360.0 * FULL_UNITS;
            
            //double set_point = (((position) + 180) / 360.0) * FULL_UNITS;
            double delta = fmod(set_point - current_position, FULL_UNITS);

            //Calculating Shortest Distance
            if(fabs(delta) > 6553.6)     {

                delta -= copysign(13107.2, delta);
                sm->invert_value = -1.0;

            }
            else 
                sm->invert_value = 1.0;
        
            sm->steer->Set(ControlMode::Position, current_position + delta + (offset < 13107.2 ? -offset : offset));
        }


        std::shared_ptr<WPI_TalonFX> drive;
        std::shared_ptr<WPI_TalonFX> steer;
        std::shared_ptr<ctre::phoenix::sensors::CANCoder> encoder;
        double offset;
        
        std::string module_name;
        double invert_value;    
        

    };

    class SwerveDrive : public frc2::SubsystemBase {
    public:
        SwerveDrive();
        ~SwerveDrive();

        virtual void Periodic();

        // Gyro
        inline double getHeading() {return m_gyro->GetFusedHeading();}
        inline double getYaw() { return m_gyro->GetYaw(); }
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

        std::shared_ptr<SwerveModule> m_lf;
        std::shared_ptr<SwerveModule> m_la;
        std::shared_ptr<SwerveModule> m_rf;
        std::shared_ptr<SwerveModule> m_ra;

    private:
        std::shared_ptr<AHRS> m_gyro;
        double m_heading_offset;
        DriveMode m_mode;
        


        bool m_drive_brake_on;

        double m_db;
        double m_speed;

        bool m_is_steering_zeroed;
    };
}

