#include "T34AnalogEncoder.h"
#include "RobotContainer.h"



T34AnalogEncoder::T34AnalogEncoder(int id)
    : frc::AnalogEncoder(id) {

}

double T34AnalogEncoder::GetDegrees() {
    double armdegrees = GetAbsolutePosition() * 360.0;
    if (armdegrees >= 0.0 && armdegrees <= 50.0)
    {
        armdegrees = 50.0-armdegrees;
    }
    else
    {
        armdegrees = fabs(armdegrees - 360.0) + 50.0;
    }

    return armdegrees;
}