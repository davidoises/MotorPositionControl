#ifndef MOTOR_CONTROLLER_HEADER
#define MOTOR_CONTROLLER_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA (5000.0f)

/****************************************************************************************
 *                                          T Y P E S
 ****************************************************************************************/

struct motor_controller_state_S
{
  float current_command;        // Current command coming out of the controller
  float position_reference;     // Reference for the position controller
};

struct motor_sensing_vars_S
{
  float motor_angle;           // Measured motor angle in degrees
  float motor_speed;           // Measured motor speed in degrees per second

  // Low pass filtered version of the measurements
  float motor_angle_filtered;
  float motor_speed_filtered;
};

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

/**
 * @brief      Runs a simple PI controller to regulate the position of the motor
 *
 * @param[in]  sensing_vars  Pointer to sensing variables struct
 *
 * @return     The current command coming out of the PI controller
 */
float position_controller(const struct motor_sensing_vars_S* sensing_vars, const float position_reference);

#endif //MOTOR_CONTROLLER_HEADER
