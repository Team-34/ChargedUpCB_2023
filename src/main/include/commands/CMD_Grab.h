#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

#include "RobotContainer.h"


namespace t34 {

    class CMD_Grab :
    public frc2::CommandHelper<frc2::CommandBase, CMD_Grab> 
    {
        double arm_ext_setpoint{};
        double current_arm_ext{};

        CMD_Grab(double arm_ext);

        virtual void Initialize();
        
        virtual void Execute();

        virtual bool IsFinished();
        
        virtual void End();

    };

}