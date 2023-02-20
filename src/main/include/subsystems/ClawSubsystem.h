#pragma once

namespace t34
{
  double EncoderToDegree(double full_rot_encoder, double current_rot_enconder);
  double CorrectionValue(double arm, double wrist);
  void WristStabilize(double wrist_degrees, double arm_degrees);
}