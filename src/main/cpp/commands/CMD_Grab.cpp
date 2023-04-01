#include "commands/CMD_Grab.h"
#include "RobotContainer.h"
#include "Constants.h"

namespace t34 {

    CMD_Grab::CMD_Grab(double arm_ext_setpoint) :
    arm_ext_setpoint(arm_ext_setpoint) {};

    void CMD_Grab::Initialize() 
    {
        auto rc = RobotContainer::get();
        rc->armSub.p_grip_solenoid->Set(false);
    };
    
    void CMD_Grab::Execute() 
    {
        auto rc = RobotContainer::get();

        rc->armSub.m_wrist_y.Set(rc->armSub.wrist_y_pid.Calculate(rc->armSub.wrist_y_degrees, 0.0));

        if (current_arm_ext <= arm_ext_setpoint)
            rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, 0.2);
        else if (current_arm_ext >= arm_ext_setpoint)
            rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, -0.2);
        else
            rc->armSub.m_arm_ext.Set(ControlMode::PercentOutput, 0.0);
    };

    bool CMD_Grab::IsFinished() 
    {
        auto rc = RobotContainer::get();

        rc->armSub.p_grip_solenoid->Set(true);

        return false;
    };

    void CMD_Grab::End() {};
}