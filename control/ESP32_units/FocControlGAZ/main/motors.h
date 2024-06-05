#pragma once
#include "SimpleFOC.h"

extern BLDCMotor motorX;
extern BLDCMotor motorY;

extern MagneticSensorSSC sensorX;
extern MagneticSensorSSC sensorY;

void initMotorsSpi();
void initMotorY();
void initMotorX();
int getXPosition();
int getYPosition();