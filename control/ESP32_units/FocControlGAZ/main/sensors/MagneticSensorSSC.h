#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "driver/spi_master.h"

/**
 * This sensor has been tested with AS5600 running in 'analog mode'.  This is where output pin of AS6000 is connected to an analog pin on your microcontroller.
 * This approach is very simple but you may more accurate results with MagneticSensorI2C if that is also supported as its skips the DAC -> ADC conversion (ADC supports MagneticSensorI2C)
 */
class MagneticSensorSSC : public Sensor {
public:
    /**
     * MagneticSensorAnalog class constructor
     * @param spiHandler  spi handler
     */
    MagneticSensorSSC(spi_device_handle_t* spiHandler);


    /** sensor initialise pins */
    void init();

    int pinAnalog; //!< encoder hardware pin A


    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;


private:
    spi_transaction_t encTrans = {};
    spi_device_handle_t* spiHandler;
};
