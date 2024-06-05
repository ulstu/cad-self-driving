#pragma once

void http_get_task(void *pvParameters);
void relay_task(void *args);

extern int targetGear;
extern float brakeTorque;
extern bool lTurn;
extern bool rTurn;
extern bool alarmS;
extern bool signalS;
extern bool ignition;
extern bool starter;