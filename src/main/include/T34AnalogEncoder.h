#pragma once

#include <frc/AnalogEncoder.h>

class T34AnalogEncoder : public frc::AnalogEncoder {
public:

    T34AnalogEncoder(int id);

    double GetDegrees();
};