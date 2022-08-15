/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "encoder_sensing.h"
#include <SPI.h>

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define SPI_BUS_FREQUENCY_HZ (8000000)

#define ANGLE_FULL_SCALE_DEG (360.0f)
#define ANGLE_DATA_LENGTH_BITS (16)
#define ANGLE_RESOLUTION_DEG_PER_BIT ( ANGLE_FULL_SCALE_DEG/((float)(2<<(ANGLE_DATA_LENGTH_BITS-1)) ) )

/****************************************************************************************
 *                       P R I V A T E   D A T A   D E F I N I T I O N S
 ****************************************************************************************/

// SPI bus objects
SPISettings SPI_bus_config = SPISettings(SPI_BUS_FREQUENCY_HZ, MSBFIRST, SPI_MODE0);
SPIClass* encoder_sensor_SPI_handle = new SPIClass(VSPI);

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

/**
 * @brief      Reads the uint16 raw angle from the encoder sensor through SPI
 *
 * @return     Raw uint16 encoder angle
 */
static uint16_t encoder_sensing_read_raw_angle_UI16()
{
  // Start SPI transmission. Uses the SPI settings object for correct mode operation
  digitalWrite(SS, LOW);
  encoder_sensor_SPI_handle->beginTransaction(SPI_bus_config);

  // Read 16-bit unsigned angle
  const uint16_t angle = encoder_sensor_SPI_handle->transfer16(0x0000);

  // End SPI transmission
  encoder_sensor_SPI_handle->endTransaction();
  digitalWrite(SS, HIGH);

  return angle;
}

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

void encoder_sensing_start_bus()
{
  // Starts the SPI bus
  encoder_sensor_SPI_handle->begin();
  pinMode(SS, OUTPUT);
}

float encoder_sensing_get_angle()
{
  // Used to keep track of absolute angle
  static float previousAngle = 0.0f;
  static float final_angle = 0.0f;

  const uint16_t raw_angle = encoder_sensing_read_raw_angle_UI16();

  // Get the original measured angle in degrees
  const float angle = raw_angle * ANGLE_RESOLUTION_DEG_PER_BIT;

  // Need to keep track fo the angle difference each iteration
  // The following logic deals with jumps between 0 and 360 degrees
  float updated_angle = angle;
  if ( previousAngle > 340 && angle < 20 )
  {
    updated_angle = angle + 360.0f;
  }
  else if ( angle > 340 && previousAngle < 20 )
  {
    updated_angle = angle - 360.0f; 
  }

  // To generate an absolute angle we add the angle difference to a state variable
  // i.e. we integrate the difference
  const float angle_difference = updated_angle - previousAngle;
  final_angle += angle_difference;

  previousAngle = angle;

  return final_angle;
}
