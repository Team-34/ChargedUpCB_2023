#include "subsystems/SwerveDrive.h"
#include <frc/smartdashboard/SmartDashboard.h>

namespace t34 {

    SwerveDrive::SwerveDrive() 
        : m_gyro(new AHRS(frc::SPI::Port::kMXP))
        , m_db(0.1)
        , m_speed(1.0)
        , m_lf(new SwerveModule("Left_Front",  ID_LEFT_FWD_DRIVE, ID_LEFT_FWD_STEER,  -1.0, ID_ENCODER_LEFT_FWD, LF_STEER_OFFSET))
        , m_la(new SwerveModule("Left Back",   ID_LEFT_AFT_DRIVE, ID_LEFT_AFT_STEER,  -1.0, ID_ENCODER_LEFT_AFT, LA_STEER_OFFSET))
        , m_rf(new SwerveModule("Right_Front", ID_RIGHT_FWD_DRIVE, ID_RIGHT_FWD_STEER, 1.0, ID_ENCODER_RIGHT_FWD, RF_STEER_OFFSET))
        , m_ra(new SwerveModule("Right_Back",  ID_RIGHT_AFT_DRIVE, ID_RIGHT_AFT_STEER, 1.0, ID_ENCODER_RIGHT_AFT, RA_STEER_OFFSET))
        , m_drive_brake_on(true)
        {

        m_lf->drive->ConfigOpenloopRamp(1.0);
        m_la->drive->ConfigOpenloopRamp(1.0);
        m_rf->drive->ConfigOpenloopRamp(1.0);
        m_ra->drive->ConfigOpenloopRamp(1.0);

        SetName("SwerveDrive");

        m_heading_offset = m_gyro ? m_gyro->GetFusedHeading() : 0.0;

        setDriveMode(DriveMode::FieldOriented);

    }

    SwerveDrive::~SwerveDrive() {}

    void SwerveDrive::zeroYaw() {

        if (m_gyro != nullptr)
            m_gyro->ZeroYaw();

    }

    void SwerveDrive::toggleDriveMode() {

        if (m_mode == DriveMode::RobotCentric)
            m_mode = DriveMode::FieldOriented;
        else
            m_mode = DriveMode::RobotCentric;
            
    }

    void SwerveDrive::setDriveBrake(bool on) {

        if (on) {

            m_lf->setDriveBrake();    
            m_la->setDriveBrake();    
            m_rf->setDriveBrake();    
            m_ra->setDriveBrake();    
            m_drive_brake_on = true;

        }
        else {

            m_lf->setDriveBrake(false);    
            m_la->setDriveBrake(false);    
            m_rf->setDriveBrake(false);    
            m_ra->setDriveBrake(false);  
            m_drive_brake_on = false;

        }

    }

    void SwerveDrive::toggleDriveBrake() {

        setDriveBrake(!m_drive_brake_on);

    }

    void setSteerPosition(std::shared_ptr<SwerveModule> sm, const double& position, double offset = 0.0) {
        double current_position = sm->steer->GetSelectedSensorPosition() ;
        double set_point = ((position + 180) / 360.0) * 26214.4;
//        double set_point = (((position + sm->zero_offset) + 180) / 360.0) * 26214.4;
        double delta = fmod(set_point - current_position, 26214.4);
        
        //Calculating Shortest Distance
        if(fabs(delta) > 6553.6)     {

            delta -= copysign(13107.2, delta);
            sm->invert_value = -1.0;

        }
        else 
            sm->invert_value = 1.0;
       
        sm->steer->Set(ControlMode::Position, current_position + delta);

    }

    void SwerveDrive::drive(double x, double y, double r) {

        //Deadband
        if (fabs(x) < m_db && fabs(y) < m_db && fabs(r) < m_db) {

            m_lf->drive->Set(ControlMode::PercentOutput, 0.0);
            m_la->drive->Set(ControlMode::PercentOutput, 0.0);
            m_rf->drive->Set(ControlMode::PercentOutput, 0.0);
            m_ra->drive->Set(ControlMode::PercentOutput, 0.0);
            return;

        }

        x *= -1.0;
        y *= -1.0;

        frc::SmartDashboard::PutNumber("X", x);
        frc::SmartDashboard::PutNumber("Y", y);
        frc::SmartDashboard::PutNumber("R", r);
        
        if (m_mode == DriveMode::FieldOriented && m_gyro != nullptr) { 
              
            double gyro_radians = deg_to_rad(m_gyro->GetYaw());
            double temp_y = y * cos(gyro_radians) + -x * sin(gyro_radians);
            x = y * sin(gyro_radians) + x * cos(gyro_radians);
            y = temp_y;       
            frc::SmartDashboard::PutNumber("FO_X", x);
            frc::SmartDashboard::PutNumber("FO_Y", y);   

        }
        
        
        double a = (x - r) * FRAME_LENGTH_DIV_RATIO();
        double b = (x + r) * FRAME_LENGTH_DIV_RATIO();
        double c = (y - r) * FRAME_WIDTH_DIV_RATIO();
        double d = (y + r) * FRAME_WIDTH_DIV_RATIO();

        setSteerPosition(m_lf, rad_to_deg(atan2(b, c)));
        setSteerPosition(m_la, rad_to_deg(atan2(a, c)));
        setSteerPosition(m_rf, rad_to_deg(atan2(b, d)));
        setSteerPosition(m_ra, rad_to_deg(atan2(a, d)));


        double lf_drive_output = sqrt(sqr(b) + sqr(c)); 
        double la_drive_output = sqrt(sqr(a) + sqr(c)); 
        double rf_drive_output = sqrt(sqr(b) + sqr(d)); 
        double ra_drive_output = sqrt(sqr(a) + sqr(d)); 

        double do_max = std::max(std::initializer_list<double>(
            { lf_drive_output, la_drive_output, rf_drive_output, ra_drive_output } ));

        if (do_max > 1.0) {

            lf_drive_output /= do_max;
            la_drive_output /= do_max;
            rf_drive_output /= do_max;
            ra_drive_output /= do_max;

        }

        m_lf->drive->Set(ControlMode::PercentOutput,   lf_drive_output * m_lf->invert_value * m_speed);
        m_la->drive->Set(ControlMode::PercentOutput, la_drive_output * m_la->invert_value * m_speed);
        m_rf->drive->Set(ControlMode::PercentOutput, rf_drive_output * m_rf->invert_value * m_speed);
        m_ra->drive->Set(ControlMode::PercentOutput, ra_drive_output * m_ra->invert_value * m_speed);

    }

    void SwerveDrive::shieldWall() {

        m_lf->drive->Set(ControlMode::PercentOutput, 0.0);
        m_la->drive->Set(ControlMode::PercentOutput, 0.0);
        m_rf->drive->Set(ControlMode::PercentOutput, 0.0);
        m_ra->drive->Set(ControlMode::PercentOutput, 0.0);

        setSteerPosition(m_lf, _315_DEG);
        setSteerPosition(m_la, _135_DEG);
        setSteerPosition(m_rf, _135_DEG);
        setSteerPosition(m_ra, _315_DEG);

    }

    void SwerveDrive::toggleSpeed() {
        
        if (m_speed < 1.0) {

            m_speed = 1.0;
            frc::SmartDashboard::PutBoolean("High Speed", true);

        }
        else {

            m_speed = 0.3;
            frc::SmartDashboard::PutBoolean("High Speed", false);

        }

    }
    
    void SwerveDrive::Periodic() {}

    void SwerveDrive::resetOdometer() {

        m_lf->drive->SetSelectedSensorPosition(0.0);
        m_la->drive->SetSelectedSensorPosition(0.0);
        m_rf->drive->SetSelectedSensorPosition(0.0);
        m_ra->drive->SetSelectedSensorPosition(0.0);

    }

    double SwerveDrive::getOdometer() {

        return (fabs(m_lf->drive->GetSelectedSensorPosition()) + 
                fabs(m_la->drive->GetSelectedSensorPosition()) + 
                fabs(m_rf->drive->GetSelectedSensorPosition()) + 
                fabs(m_ra->drive->GetSelectedSensorPosition())) / 4;

    }

    void SwerveDrive::putTelemetry() {

        frc::SmartDashboard::PutNumber("Heading", getHeading());
        frc::SmartDashboard::PutNumber("Yaw", getYaw());

        frc::SmartDashboard::PutBoolean("High Speed", true);
        frc::SmartDashboard::PutNumber("Speed", m_speed);

        double lf = m_lf->steer->GetSelectedSensorPosition();
        double la = m_la->steer->GetSelectedSensorPosition();
        double rf = m_rf->steer->GetSelectedSensorPosition();
        double ra = m_ra->steer->GetSelectedSensorPosition();

        frc::SmartDashboard::PutNumber(m_lf->module_name + " CSPR", lf);
        frc::SmartDashboard::PutNumber(m_la->module_name + " CSPR", la);
        frc::SmartDashboard::PutNumber(m_rf->module_name + " CSPR", rf);
        frc::SmartDashboard::PutNumber(m_ra->module_name + " CSPR", ra);

        frc::SmartDashboard::PutNumber(m_lf->module_name + " CSP", (lf + 180) / 360.0);
        frc::SmartDashboard::PutNumber(m_la->module_name + " CSP", (la + 180) / 360.0);
        frc::SmartDashboard::PutNumber(m_rf->module_name + " CSP", (rf + 180) / 360.0);
        frc::SmartDashboard::PutNumber(m_ra->module_name + " CSP", (ra + 180) / 360.0);

        frc::SmartDashboard::PutNumber(m_lf->module_name + " CDP", m_lf->drive->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber(m_la->module_name + " CDP", m_la->drive->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber(m_rf->module_name + " CDP", m_rf->drive->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber(m_ra->module_name + " CDP", m_ra->drive->GetSelectedSensorPosition());

        frc::SmartDashboard::PutNumber(m_lf->module_name + " CStP", m_lf->steer->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber(m_la->module_name + " CStP", m_la->steer->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber(m_rf->module_name + " CStP", m_rf->steer->GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber(m_ra->module_name + " CStP", m_ra->steer->GetSelectedSensorPosition());
  
        frc::SmartDashboard::PutString("Drive Mode ", m_mode == DriveMode::RobotCentric ? "Robot Centric" : "Field Oriented");        

    }

    void SwerveDrive::zeroIntegratedEncoders()
    {
        m_lf->zero_offset = m_lf->encoder.GetAbsolutePosition() * 72.81777;
        m_la->zero_offset = m_la->encoder.GetAbsolutePosition() * 72.81777;
        m_rf->zero_offset = m_rf->encoder.GetAbsolutePosition() * 72.81777;
        m_ra->zero_offset = m_ra->encoder.GetAbsolutePosition() * 72.81777;

        m_lf->steer->SetSelectedSensorPosition(m_lf->steer->GetSelectedSensorPosition() - m_lf->zero_offset);
        m_la->steer->SetSelectedSensorPosition(m_la->steer->GetSelectedSensorPosition() - m_la->zero_offset);
        m_rf->steer->SetSelectedSensorPosition(m_rf->steer->GetSelectedSensorPosition() - m_rf->zero_offset);
        m_ra->steer->SetSelectedSensorPosition(m_ra->steer->GetSelectedSensorPosition() - m_ra->zero_offset);
    }

  void SwerveDrive::zeroDrive()
  {
        m_lf->drive->SetSelectedSensorPosition(0.0);
        m_la->drive->SetSelectedSensorPosition(0.0);
        m_rf->drive->SetSelectedSensorPosition(0.0);
        m_ra->drive->SetSelectedSensorPosition(0.0);
  }

}
