/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "encoder_sensing.h"
#include <SPI.h>

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define ENCODER_CHANNEL_A_PIN 13  
#define ENCODER_CHANNEL_B_PIN 15

#define ANGLE_FULL_SCALE_DEG (360.0f)
#define ANGLE_DATA_LENGTH_BITS (12)
#define ANGLE_RESOLUTION_DEG_PER_BIT ( ANGLE_FULL_SCALE_DEG/((float)(2<<(ANGLE_DATA_LENGTH_BITS-1)) ) )

/****************************************************************************************
 *                       P R I V A T E   D A T A   D E F I N I T I O N S
 ****************************************************************************************/

static volatile int32_t counter = 0U;

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

static void IRAM_ATTR encoder_sensing_process_interrupts()
{
  static uint8_t previous_state = 0U;
  uint8_t state = 0U;

  const uint8_t A = digitalRead(ENCODER_CHANNEL_A_PIN);
  const uint8_t B = digitalRead(ENCODER_CHANNEL_B_PIN);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;

  switch (state)
  {
    case 1:
    {
      if (previous_state == 2) counter++;
      if (previous_state == 4) counter--;
      break;
    }
    case 2:
    {
      if (previous_state == 1) counter--;
      if (previous_state == 3) counter++;
      break;
    }
    case 3:
    {
      if (previous_state == 2) counter--;
      if (previous_state == 4) counter++;
      break;
    }
    default:
    {
      if (previous_state == 1) counter++;
      if (previous_state == 3) counter--;
    }
  }

  previous_state = state;
}

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

void encoder_sensing_start()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_PIN), encoder_sensing_process_interrupts, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_B_PIN), encoder_sensing_process_interrupts, CHANGE);
}

float encoder_sensing_get_angle()
{

  // Get the original measured angle in degrees
  const float angle = counter * ANGLE_RESOLUTION_DEG_PER_BIT;

  return angle;
}

float encoder_sensing_get_speed(const float time_step)
{
  static int32_t previous_count = 0U;

  const int32_t counter_diff = counter - previous_count;
  previous_count = counter;

  const float speed = counter_diff * ANGLE_RESOLUTION_DEG_PER_BIT / time_step;

  return speed;
}
