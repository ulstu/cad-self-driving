#include "Gearbox.h"

static const char *TAG = "GEARBOX";

void Gearbox::set_current_gear(int target_gear) { current_gear = target_gear; }
int Gearbox::get_current_gear() { return current_gear; }
