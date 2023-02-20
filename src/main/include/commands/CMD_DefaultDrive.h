#ifndef CMD_DEFAULTDRIVE_H
#define CMD_DEFAULTDRIVE_H

#include <memory>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include "utils/T34XboxController.h"

namespace t34 {

    class DefaultDriveCommand 
        : public frc2::CommandHelper<frc2::CommandBase, DefaultDriveCommand> {

    public:
        DefaultDriveCommand(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<T34XboxController> controller);

        void Initialize();
        void Execute() override;

    private:
        std::shared_ptr<SwerveDrive> m_drive;
        std::shared_ptr<T34XboxController> m_driver_control;
        double m_driving_speed;
    };

}

#endif //CMD_DEFAULTDRIVE_H