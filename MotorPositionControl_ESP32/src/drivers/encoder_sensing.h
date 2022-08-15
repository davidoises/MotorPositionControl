#ifndef ENCODER_SENSING_HEADER
#define ENCODER_SENSING_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

/**
 * @brief      Starts the SPI bus for sensor communication
 */
void encoder_sensing_start_bus();

/**
 * @brief      Gets the raw angle from the SPI sensor.
 *
 * @return     Encoder angle in degrees in range of [0, 360]
 */
float encoder_sensing_get_angle();

#endif //ENCODER_SENSING_HEADER
