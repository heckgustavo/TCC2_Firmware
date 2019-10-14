#include "L3GD20.h"

__IO uint32_t  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;
SPI_HandleTypeDef hspi1;

void L3GD20_SetSPI(SPI_HandleTypeDef h)
{
	hspi1 = h;
}

uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;

  /* Read STATUS_REG register */
  L3GD20_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);

  return tmpreg;
}


void L3GD20_Write(uint8_t buffer, uint8_t WriteAddr)
{
  /* Chip select (CS) to begin */
  L3GD20_CS_LOW();
  uint8_t cmd[2] = {0x00 | WriteAddr, buffer};
  HAL_SPI_Transmit(&hspi1,cmd,2,L3GD20_FLAG_TIMEOUT);
  L3GD20_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the L3GD20.
  * @param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
  * @param  ReadAddr : L3GD20's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the L3GD20.
  * @retval None
  */
void L3GD20_Read(uint8_t* pBuffer,uint8_t ReadAddr, uint16_t NumByteToRead)
{
  /* Chip select (CS) to begin */
  L3GD20_CS_LOW();

  /* Format HAL command */
  uint8_t cmd[1+NumByteToRead];// = {READWRITE_CMD | WriteAddr, pBuffer[0]};

  cmd[0] = READWRITE_CMD | MULTIPLEBYTE_CMD | ReadAddr;
  for(int i = 1; i <= NumByteToRead; i++)
	  cmd[i] = DUMMY_BYTE;
  /* Transmit/Receive over SPI */
  HAL_SPI_TransmitReceive(&hspi1,cmd,pBuffer,NumByteToRead+1,L3GD20_FLAG_TIMEOUT);

  L3GD20_CS_HIGH();
}

/**
 * @brief Reads the x-axis angular rate data and returns it as a float
 * @param None
 * @retval X-axis angular rate
 */
float L3GD20_GetAngularRateX(float sensitivity)
{
	float ang = 0;
	uint8_t x_buffer[3] = {0,0,0};
	int16_t xdata_raw = 0;

	L3GD20_Read(x_buffer, L3GD20_OUT_X_L_ADDR, 2);

	xdata_raw = (int16_t)( (uint16_t)(x_buffer[2] << 8) + x_buffer[1] );
	ang = (float)xdata_raw/sensitivity;

	return ang;
}

/**
 * @brief Reads the y-axis angular rate data and returns it as a float
 * @param None
 * @retval Y-axis angular rate
 */
float L3GD20_GetAngularRateY(float sensitivity)
{
	float ang = 0;
	uint8_t y_buffer[3];
	int16_t ydata_raw = 0;

	L3GD20_Read(y_buffer, L3GD20_OUT_Y_L_ADDR, 2);

	ydata_raw = (int16_t)( (uint16_t)(y_buffer[2] << 8) + y_buffer[1] );
	ang = (float)ydata_raw/sensitivity;

	return ang;
}


/**
 * @brief Reads the z-axis angular rate data and returns it as a float
 * @param None
 * @retval Z-axis angular rate
 */
float L3GD20_GetAngularRateZ(float sensitivity)
{
	float ang = 0;
	uint8_t z_buffer[3];
	int16_t zdata_raw = 0;

	L3GD20_Read(z_buffer, L3GD20_OUT_Z_L_ADDR, 2);

	zdata_raw = (int16_t)( (uint16_t)(z_buffer[2] << 8) + z_buffer[1] );
	ang = (float)zdata_raw/sensitivity;

	return ang;
}

