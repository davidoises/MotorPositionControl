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
 * @brief      Gets the angle based on the encoder counts
 *
 * @return     Encoder angle in degrees in range of [-inf, inf]
 */
float encoder_sensing_get_angle();

/**
 * @brief      Gets the speed based on the encoder counts within a give time step
 *
 * @param[in]  time_step  The time step between measurements
 *
 * @return     Encoder angular speed in deg/s
 */
float encoder_sensing_get_speed(const float time_step);

#endif //ENCODER_SENSING_HEADER
