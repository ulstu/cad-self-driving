#include "StateMachine.h"

void StateMachine::init() { current_control_mode = EStop; }
StateMachine::Mode StateMachine::get_current_control_mode() { return current_control_mode; }
