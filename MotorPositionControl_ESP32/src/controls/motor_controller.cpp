/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "motor_controller.h"

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define POSITION_CONTROLLER_KP (10.95f) //3.9125 1.95
#define POSITION_CONTROLLER_KI (30e-6f * 500.0f) //20.0f // 4.375e-6 - 2.1875e-6
#define POSITION_CONTROLLER_KD (0.25f) //0.1//0.1 - 0.01

#define POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_STATIC_FRICTION_mA (200.0f)
#define POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_DYNAMIC_FRICTION_mA (80.0f)

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
    const float error = position_reference - sensing_vars->motor_angle_filtered;

    // Proportional term calculation
    const float p_term = error * POSITION_CONTROLLER_KP;

    // Integral term calculation
    static float i_term = 0.0f;
    i_term += error * POSITION_CONTROLLER_KI;
    i_term = constrain(i_term, -POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA*1.0f/3.0f, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA*1.0f/3.0f);

    // Derivative term
    const float d_term = -sensing_vars->motor_speed_filtered * POSITION_CONTROLLER_KD;
    //const float d_term = 0.0f;

    // PI combination
    const float control_law = constrain(p_term + i_term + d_term, -POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA);

    

    //// Next steps from here are attempts to reduce oscillations and steady state error due to friction/deadzones

    // Determnie the minimum current command based on frctions effects
    // Trying to follow some high stiction level and low dynamic friction based on testing
    static float prev_command = 0.0f;
    float min_command = POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_DYNAMIC_FRICTION_mA;
    if (abs(sensing_vars->motor_speed_filtered) < 300.0f || (prev_command * control_law) < 0.0f)
    {
        min_command = POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_STATIC_FRICTION_mA;
    }

    // Adjust command to follow min command based on friction
    motor_current_command = max(abs(control_law), min_command);
    if (control_law < 0.0f)
    {
        motor_current_command *= -1.0f;
    }

    // deadband control
    if (abs(control_law) < 60.0f)
    {
        motor_current_command = 0.0f;
    }

    prev_command = motor_current_command;

    // Serial.print(control_law);
    // Serial.print(" ");
    // Serial.println(motor_current_command);

    return motor_current_command;
}
