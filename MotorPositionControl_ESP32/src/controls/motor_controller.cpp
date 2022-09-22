/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "motor_controller.h"

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define POSITION_CONTROLLER_KP (10.95f) //3.9125 1.95
#define POSITION_CONTROLLER_KI (7.5f * 0.002f) 
#define POSITION_CONTROLLER_KD (0.25f) //0.1//0.1 - 0.01

#define POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_STATIC_FRICTION_mA (200.0f) //200
#define POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_DYNAMIC_FRICTION_mA (60.0f)
#define POSITION_CONTROLLER_CURRENT_COMMAND_STATIC_DEADZONE_THRESHOLD_SPEED_DEGPERS (100.0f) //300.0f

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
    //const float control_law = constrain(p_term + i_term + d_term, -POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA);
    float control_law = position_reference;

    

    //// Next steps from here are attempts to reduce oscillations and steady state error due to friction/deadzones


    static float prev_speed = 0.0f;
    float delta_speed = sensing_vars->motor_speed_filtered - prev_speed;
    prev_speed = sensing_vars->motor_speed_filtered;

    float min_command = POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_STATIC_FRICTION_mA;
    //if (abs(sensing_vars->motor_speed_filtered) > 400.0f)
    if (abs(sensing_vars->motor_speed_super_filtered) > 200.0f)
    {
        min_command = POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_DYNAMIC_FRICTION_mA;
    }
    // float min_command = POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_DYNAMIC_FRICTION_mA;
    // if ((delta_speed * sensing_vars->motor_speed_filtered) >= 0.0f)
    // {
    //     min_command = POSITION_CONTROLLER_CURRENT_COMMAND_DEADZONE_STATIC_FRICTION_mA;
    // }

    motor_current_command = map(abs(control_law), 0.0f, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA, min_command, POSITION_CONTROLLER_MAX_CURRENT_COMMAND_mA);
    if (control_law < 0.0f)
    {
        motor_current_command *= -1.0f;
    }

    // deadband control
    // if (abs(control_law) < 20.0f)
    // {
    //     motor_current_command = 0.0f;
    // }

    // Serial.print(delta_speed);
    // Serial.print(" ");
    // Serial.print(motor_current_command);
    // Serial.print(" ");
    // Serial.print(sensing_vars->motor_speed);
    // Serial.print(" ");
    // Serial.println(motor_sensing_vars.motor_speed_filtered);
    
    // Serial.print(sensing_vars->motor_speed);
    // Serial.print(" ");
    // Serial.println(min_command);


    return motor_current_command;
}
