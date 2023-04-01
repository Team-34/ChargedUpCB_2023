// #pragma once

// #include <frc2/command/CommandBase.h>
// #include <frc2/command/CommandHelper.h>
// #include <AHRS.h>
// #include <frc/controller/PIDController.h>

// #include "subsystems/SwerveDrive.h"

//     namespace t34
//     {
//         class CMD_AutoBal
//         : public frc2::CommandHelper<frc2::CommandBase, CMD_AutoBal> 
//         {
//             public:
//                 CMD_AutoBal();
        
//                 virtual void Initialize();
        
//                 virtual void Execute();
        
//                 virtual bool IsFinished();
        
//                 virtual void End();
        
//             private:
//                 frc2::PIDController PitchPID;
//                 std::shared_ptr<AHRS> navX;
//         };
//     }