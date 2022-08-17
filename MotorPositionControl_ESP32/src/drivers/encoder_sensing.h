#ifndef ENCODER_SENSING_HEADER
#define ENCODER_SENSING_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

/**
 * @brief      Attaches interrupts and starts ISR for encoder AB inputs
 */
void encoder_sensing_start();

/**
 * @brief      Gets the raw angle from the SPI sensor.
 *
 * @return     Encoder angle in degrees in range of [0, 360]
 */
float encoder_sensing_get_angle();

#endif //ENCODER_SENSING_HEADER
