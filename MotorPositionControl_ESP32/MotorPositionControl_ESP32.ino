/*
 * @file StateSpacePendulumController
 * 
 * Zero regulation, state space controller of the angular position of reaction wheel pendulum.
 * Intended to regulate the pendulum angular positino to 0 degrees, but an utomatic setpoint
 * layer allows stability in the presence of mechanical unbalance in the pendulum.
 *
 * @author [davidoises]
 * @version  V2.0
 * @date  2022-June-10
 * @https://github.com/davidoises/ReactionWheelPendulum
 */
 
/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include "src/drivers/motor_controller_handler.h"
#include "src/drivers/encoder_sensing.h"
#include "src/controls/motor_controller.h"

/****************************************************************************************
 *                                        D E F I N E S
 ****************************************************************************************/
 
#define CONTROL_ISR_PERIOD_US 2000
#define CONTROL_ISR_FREQUENCY_HZ (1000000.0f/CONTROL_ISR_PERIOD_US)

#define COMMS_UPDATE_FREQ_HZ (100.0f)
#define COMMS_ISR_TICKS_THRESHOLD (CONTROL_ISR_FREQUENCY_HZ/COMMS_UPDATE_FREQ_HZ)

// 7 HZ Low pass filter constant
#define SENSING_LPF_CUTOFF_HZ (50.0f)
#define SENSING_LPF_ALPHA (1.0f / (1.0f + CONTROL_ISR_FREQUENCY_HZ*(1.0f/(2*PI*SENSING_LPF_CUTOFF_HZ))))

#define CONVERSION_RPM_TO_DEGPS (360.0f/60.0f)

/****************************************************************************************
 *                              G L O B A L   V A R I A B L E S
 ****************************************************************************************/
 
struct motor_controller_state_S motor_controller_state = {0};
struct motor_sensing_vars_S motor_sensing_vars = {0};

// Control ISR timer
hw_timer_t * sampling_timer = NULL;

// Flag to run code ISR code in main loop
volatile uint8_t isr_tick = 0U;

// Flat to run comms tasks at the comms update rate
volatile uint8_t comms_tick = 0U;

// Divides the isr clock for tasks that can run slower
volatile uint16_t isr_comms_divider = 0U;

/****************************************************************************************
 *                               I S R   C A L L B A C K S                      
 ****************************************************************************************/

void IRAM_ATTR sampling_isr() {
  // Trigger a flag for other code to run at 100Hz
  isr_tick = 1U;

  // Update the comms tick once the divider counter reaches the intended frequency
  isr_comms_divider++;
  //if(isr_comms_divider == COMMS_ISR_TICKS_THRESHOLD) //TODO
  if(isr_comms_divider == 5)
  {
    // Set the comms tick
    comms_tick = 1U;
    
    isr_comms_divider = 0U;
  }
}

/****************************************************************************************
 *                        A R D U I N O   B A S E   F U N C T I O N S                  
 ****************************************************************************************/

void setup()
{
  // Start primary serial for user communications  
  Serial.begin(115200);

  // User guard to prevent unintended/immediate oepration when booting
  Serial.println("Press anything to start execution");
  while(Serial.available() == 0) {}

  // Print some warning for the user
  Serial.println("Starting program in 2 second");

  // Init IST for encoder AB input processing
  encoder_sensing_start();

  // Initializes motor controller
  motor_controller_handler_init();

  delay(2000);

  // Set sampling trigger to CONTROL_ISR_PERIOD_US microseconds
  isr_tick = 0;
  sampling_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(sampling_timer, &sampling_isr, true);
  timerAlarmWrite(sampling_timer, CONTROL_ISR_PERIOD_US, true);
  timerAlarmEnable(sampling_timer);

}

void loop()
{
  // Code running at ISR rate
  if (isr_tick)
  {
    static uint32_t previousT;
    uint32_t currentT = micros();
    float dt = (currentT - previousT);
    previousT = currentT;
    
    // Controls code was to slow to run directly in the ISR so moved to main loop
    // The slowliness is due to the UART communication for the motor controller.
    
    // 1. Gather sensgin variables
    motor_sensing_vars.motor_angle = encoder_sensing_get_angle();
    motor_sensing_vars.motor_speed = motor_controller_handler_get_speed() * -CONVERSION_RPM_TO_DEGPS; // Getting speed in Deg/s
    
    // Low pass filter mesurements from sensors
    motor_sensing_vars.motor_angle_filtered = (1.0f-SENSING_LPF_ALPHA) * motor_sensing_vars.motor_angle_filtered + SENSING_LPF_ALPHA * motor_sensing_vars.motor_angle;
    motor_sensing_vars.motor_speed_filtered = (1.0f-SENSING_LPF_ALPHA) * motor_sensing_vars.motor_speed_filtered + SENSING_LPF_ALPHA * motor_sensing_vars.motor_speed;
    
    // 2. Run the control loops
    motor_controller_state.current_command = position_controller(&motor_sensing_vars, motor_controller_state.position_reference);
    
    // 3. Send the actual command to the actuator
    //motor_controller_handler_set_current(-1.0f*motor_controller_state.current_command);
    //motor_controller_handler_set_current(-200);
    
    isr_tick = 0U;
  }

  // Code running at COMMs rate
  if (comms_tick)
  {
    // Updating the position reference here
    motor_controller_state.position_reference = 180.0f;
    
    // Data for serial plotter
    Serial.println(motor_sensing_vars.motor_angle);//_filtered);
    //Serial.print(" ");
    //Serial.println(motor_sensing_vars.motor_speed_filtered);
    //Serial.print(" ");
    //Serial.println(dt);

    comms_tick = 0U;
  }
}
