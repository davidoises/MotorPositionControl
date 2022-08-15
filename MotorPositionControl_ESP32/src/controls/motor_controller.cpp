/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "motor_controller.h"

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define POSITION_CONTROLLER_KP (0.0f)
#define POSITION_CONTROLLER_KI (0.0f)
#define POSITION_CONTROLLER_KD (0.0f)

#define POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA (5000.0f)
#define POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_mA (50.0f)

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

float position_controller(const struct motor_sensing_vars_S* sensing_vars, const float position_reference)
{
    return 0.0f;
}
