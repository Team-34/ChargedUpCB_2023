#include "commands/CMD_Drop.h"

CMD_Drop::CMD_Drop(double arm_ext) :
current_arm_ext(arm_ext){}

void CMD_Drop::Initialize() {}

void CMD_Drop::Execute() {
  
}

void CMD_Drop::End(bool interrupted) {}

bool CMD_Drop::IsFinished() {
  return false;
}
