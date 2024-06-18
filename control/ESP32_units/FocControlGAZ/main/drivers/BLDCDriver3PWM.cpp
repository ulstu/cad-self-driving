#include "BLDCDriver3PWM.h"
#include "driver/gpio.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
  // enable_pin the driver - if enable_pin pin available
  if ( _isset(enableA_pin) ) gpio_set_level((gpio_num_t)enableA_pin, enable_active_high);
  if ( _isset(enableB_pin) ) gpio_set_level((gpio_num_t)enableB_pin, enable_active_high);
  if ( _isset(enableC_pin) ) gpio_set_level((gpio_num_t)enableC_pin, enable_active_high);
  // set zero to PWM
  setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available

  if ( _isset(enableA_pin) ) gpio_set_level((gpio_num_t)enableA_pin, !enable_active_high);
  if ( _isset(enableB_pin) ) gpio_set_level((gpio_num_t)enableB_pin, !enable_active_high);
  if ( _isset(enableC_pin) ) gpio_set_level((gpio_num_t)enableC_pin, !enable_active_high);

}

// init hardware pins
int BLDCDriver3PWM::init() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = 1 << pwmA | 1 << pwmB | 1 << pwmC;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  
  if( _isset(enableA_pin)) io_conf.pin_bit_mask |= 1 << enableA_pin;
  if( _isset(enableB_pin)) io_conf.pin_bit_mask |= 1 << enableB_pin;
  if( _isset(enableC_pin)) io_conf.pin_bit_mask |= 1 << enableC_pin;

  gpio_config(&io_conf);


  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}



// Set voltage to the pwm pin
void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
  // disable if needed
  if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
    gpio_set_level((gpio_num_t)enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
    gpio_set_level((gpio_num_t)enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
    gpio_set_level((gpio_num_t)enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
  }
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}
