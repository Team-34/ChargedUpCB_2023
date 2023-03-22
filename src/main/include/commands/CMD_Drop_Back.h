#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

namespace t34 {

 class CMD_Drop_Back
     : public frc2::CommandHelper<frc2::CommandBase, CMD_Drop_Back> {

  public:
   std::string placement{};

   double setpoint{};
   // Placement can be: "TOP CUBE", "MID CUBE"

   CMD_Drop_Back(std::string placement);

   void Initialize() override;

   void Execute() override;

   void End(bool interrupted) override;

   bool IsFinished() override;
 };
}