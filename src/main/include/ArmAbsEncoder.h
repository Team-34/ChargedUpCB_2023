#pragma once

#include <frc/AnalogEncoder.h>

class ArmAbsEncoder : public frc::AnalogEncoder {
public:

    ArmAbsEncoder(int id);

    double GetDegrees();
};