/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "motor_controller.h"

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define POSITION_CONTROLLER_KP (1.0f)
#define POSITION_CONTROLLER_KI (0.000005f * 500.0f) //0.000001
#define POSITION_CONTROLLER_KD (0.01f)

#define POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA (5000.0f)
#define POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_mA (0.0f)//(50.0f)

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

float position_controller(const struct motor_sensing_vars_S* sensing_vars, const float position_reference)
{
    // Final command out of controller
    float motor_current_command = 0.0f;

    // Error
    const float error = position_reference - sensing_vars->motor_angle;//_filtered;

    // Proportional term calculation
    const float p_term = error * POSITION_CONTROLLER_KP;

    // Integral term calculation
    static float i_term = 0.0f;
    i_term += error * POSITION_CONTROLLER_KI;
    i_term = constrain(i_term, -POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA);

    // Derivative term
    const float d_term = sensing_vars->motor_speed_filtered * -POSITION_CONTROLLER_KD;

    // PI combination
    const float control_law = constrain(p_term + i_term + d_term, -POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA);

    // Adjust the output command with a linear mapping to account for motor controller deadzone
    // converts from a var in range [A, B] to a new range [C, D]
    const float mapped_control = map(abs(control_law), 0.0f, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA, POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_mA, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA);
    if(control_law < 0.0f)
    {
        motor_current_command = -1.0f*mapped_control;
    }
    else
    {
        motor_current_command = mapped_control;
    }

    return motor_current_command;
}
