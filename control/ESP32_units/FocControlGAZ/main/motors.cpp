#include "motors.h"
#include "nvstorage.h"
#include "Gearbox.h"

spi_device_handle_t encoderXSpi;
spi_device_handle_t encoderYSpi;

MagneticSensorSSC sensorX = MagneticSensorSSC(&encoderXSpi);
MagneticSensorSSC sensorY = MagneticSensorSSC(&encoderYSpi);

BLDCMotor motorX = BLDCMotor(15);
BLDCMotor motorY = BLDCMotor(15);

BLDCDriver3PWM driverY = BLDCDriver3PWM(5, 7, 15, 18);
BLDCDriver3PWM driverX = BLDCDriver3PWM(4, 6, 16, 17);


void initMotorsSpi() {
    spi_bus_config_t encoderBuscfg= {};
    encoderBuscfg.miso_io_num = 35,
    encoderBuscfg.mosi_io_num = 36;
    encoderBuscfg.sclk_io_num = 37;
    encoderBuscfg.quadwp_io_num = -1;
    encoderBuscfg.quadhd_io_num = -1;
    encoderBuscfg.max_transfer_sz = 32;  
    

    spi_device_interface_config_t encoderX = {};
    encoderX.command_bits = 0;
    encoderX.address_bits = 0;
    encoderX.dummy_bits = 0;
    encoderX.clock_speed_hz = 1 * 1000 * 1000;
    encoderX.spics_io_num = 39,
    encoderX.mode = 0;
    encoderX.queue_size = 1;

    spi_device_interface_config_t encoderY = {};
    encoderY.command_bits = 0;
    encoderY.address_bits = 0;
    encoderY.dummy_bits = 0;
    encoderY.clock_speed_hz = 1 * 1000 * 1000;
    encoderY.spics_io_num = 38,
    encoderY.mode = 0;
    encoderY.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &encoderBuscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &encoderX, &encoderXSpi));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &encoderY, &encoderYSpi));
}

void initMotorY() {
    loadYMotorParams();
    motorY.setPostions(11.10, 10.16, 9.45, 0.075);
    sensorY.init();
    motorY.linkSensor(&sensorY);
    driverY.voltage_power_supply = 12;
    driverY.init();
    motorY.linkDriver(&driverY);
    motorY.voltage_sensor_align = 5;

    // motorY.PID_velocity.P = motorYparams.velLimitHard;
    // motorY.PID_velocity.I = motorYparams.velIntegral;
    // motorY.PID_velocity.D = motorYparams.velDiff;
    // motorY.PID_velocity.output_ramp = motorYparams.velProp;
    // motorY.PID_velocity.limit = motorYparams.velLimit;
    // motorY.LPF_velocity.Tf = motorYparams.velFilter;
    
    // motorY.P_angle.P = motorYparams.angleProp;
    // motorY.P_angle.limit = motorYparams.angleLimit;

    // motorY.voltage_limit = motorYparams.voltageLimit;
    motorY.voltage_limit = 12;
    motorY.controller = MotionControlType::angle;
    motorX.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motorY.init();
    if (motorYparams.calibrate) {
        motorY.initFOC();
        motorYparams.encoderAngle = motorY.zero_electric_angle;
        motorYparams.inverseEncoder = (motorY.sensor_direction == Direction::CCW ? true : false);
    } else {
        motorY.initFOC(motorYparams.encoderAngle, motorYparams.inverseEncoder ? Direction::CCW : Direction::CW);
    }
    
    motorY.disable();
}

void initMotorX() {
    loadXMotorParams();
    motorX.setPostions(10.7, 10.31, 9.9, 0.075);
    sensorX.init();
    motorX.linkSensor(&sensorX);
    driverX.voltage_power_supply = 12;
    driverX.init();
    motorX.linkDriver(&driverX);

    motorX.voltage_sensor_align = 5;


    // motorX.PID_velocity.P = motorXparams.velLimitHard;
    // motorX.PID_velocity.I = motorXparams.velIntegral;
    // motorX.PID_velocity.D = motorXparams.velDiff;
    // motorX.PID_velocity.output_ramp = motorXparams.velProp;
    // motorX.PID_velocity.limit = motorXparams.velLimit;
    // motorX.LPF_velocity.Tf = motorXparams.velFilter;
    
    // motorX.P_angle.P = motorXparams.angleProp;
    // motorX.P_angle.limit = motorXparams.angleLimit;
    motorX.velocity_limit = motorXparams.velLimitHard;

    //motorX.voltage_limit = motorXparams.voltageLimit;

    motorX.voltage_limit = 12;
    motorX.controller = MotionControlType::angle;
    motorX.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motorX.init();
    if (motorXparams.calibrate) {
        motorX.initFOC();
        motorXparams.encoderAngle = motorX.zero_electric_angle;
        motorXparams.inverseEncoder = (motorX.sensor_direction == Direction::CCW ? true : false);
    } else {
        motorX.initFOC(motorXparams.encoderAngle, motorXparams.inverseEncoder ? Direction::CCW : Direction::CW);
    }
    
    motorX.disable();
}
