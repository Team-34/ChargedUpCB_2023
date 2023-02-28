#include "commands/CMD_Grab.h"
#include "RobotContainer.h"
#include "Constants.h"

namespace t34 {

    CMD_Grab::CMD_Grab(double arm_ext_setpoint) :
    arm_ext_setpoint(arm_ext_setpoint) {};

    void CMD_Grab::Initialize() {};
    
    void CMD_Grab::Execute() 
    {
              auto rc = RobotContainer::get();

        if (current_arm_ext <= arm_ext_setpoint)
            rc->m_arm_ext->Set(0.2);
        else
            rc->m_arm_ext->Set(0.0);

        rc->wrist_y_pid.Calculate(rc->wrist_degrees, -90.0);
        
    };

    bool CMD_Grab::IsFinished() {};

    void CMD_Grab::End() {};
}