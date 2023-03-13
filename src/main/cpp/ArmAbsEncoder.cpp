#include "ArmAbsEncoder.h"
#include "RobotContainer.h"



ArmAbsEncoder::ArmAbsEncoder(int id)
    : frc::AnalogEncoder(id) {

}

double ArmAbsEncoder::GetDegrees() {
    double armdegrees = GetAbsolutePosition() * 360.0;
/*
    if (armdegrees >= 0.0 && armdegrees <= 50.0)
        armdegrees = 50.0-armdegrees;
    else
        armdegrees = fabs(armdegrees - 360.0) + 50.0;
*/
    return armdegrees;
}