#pragma once

#include <memory>

#include <AHRS.h>

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/filter/SlewRateLimiter.h>


namespace t34 {

    enum class DriveMode {
        RobotCentric,
        FieldOriented
    };



    class SwerveModule {
    public:
        SwerveModule(std::string name, const int drive_id, const int steer_id, int invert, int encoder_id, double offset) 
            : module_name(name)
            , drive(new WPI_TalonFX(drive_id))
            , steer(new WPI_TalonFX(steer_id))
            , invert_value(invert)
            , encoder(encoder_id)
            , zero_offset(offset) {

            // Configure Drive Motor
            drive->ConfigFactoryDefault();
            drive->SetNeutralMode(NeutralMode::Brake);
            setDriveBrake(true);
            // ??? SETTINGS FOR DRIVE ???

            // Configure Steer Motor
            steer->ConfigFactoryDefault();            
            steer->SetNeutralMode(NeutralMode::Coast);
            steer->ConfigFeedbackNotContinuous(false);
            steer->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
            steer->ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180);
            steer->ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero);
            steer->Config_kP(0, 0.2, 0.0);
            steer->Config_kD(0, 0.0, 0.0);

            encoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
            encoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
            encoder.ConfigMagnetOffset(offset);
            encoder.SetPositionToAbsolute();     
            
        }

        inline void setDriveBrake(bool on = true) { drive->SetNeutralMode(on ? NeutralMode::Brake : NeutralMode::Coast); }


        std::shared_ptr<WPI_TalonFX> drive;
        std::shared_ptr<WPI_TalonFX> steer;
        CANCoder encoder;
        
        std::string module_name;
        double invert_value;
        double zero_offset;   

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
        void shieldWall();
        void toggleSpeed();
        
        // Odometer
        void resetOdometer();
        double getOdometer();

        // Smart Dashboard
        void putTelemetry();

        // Zeroing
        void zeroIntegratedEncoders();

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
        
    };

}

