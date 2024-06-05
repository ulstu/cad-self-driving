#pragma once
#include "BTS7960.h"
#include "BrushedMotor.h"

extern BTS7960 clutch_motor_driver;
extern BrushedMotor clutch_motor;

extern BTS7960 brake_motor_driver;
extern BrushedMotor brake_motor;

void motors_init();

void set_clutch_motor_params();

void set_brake_motor_params();