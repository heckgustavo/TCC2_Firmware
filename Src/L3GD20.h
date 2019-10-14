/*
 * L3GD20.h
 *
 *  Created on: June 3, 2017
 *      Author: Sam Gallagher
 */

#ifndef L3GD20_H_
#define L3GD20_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define L3GD20_FLAG_TIMEOUT             ((uint32_t)0x1000)

/**
  * @brief  L3GD20 SPI Interface pins
  */
#define L3GD20_SPI                       SPI1
#define L3GD20_SPI_CLK                   RCC_APB2Periph_SPI1

#define L3GD20_SPI_SCK_PIN               GPIO_PIN_5                  /* PA.05 */
#define L3GD20_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
#define L3GD20_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define L3GD20_SPI_SCK_SOURCE            GPIO_PinSource5
#define L3GD20_SPI_SCK_AF                GPIO_AF_SPI1

#define L3GD20_SPI_MISO_PIN              GPIO_PIN_6                  /* PA.6 */
#define L3GD20_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
#define L3GD20_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define L3GD20_SPI_MISO_SOURCE           GPIO_PinSource6
#define L3GD20_SPI_MISO_AF               GPIO_AF_SPI1

#define L3GD20_SPI_MOSI_PIN              GPIO_PIN_7                  /* PA.7 */
#define L3GD20_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
#define L3GD20_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define L3GD20_SPI_MOSI_SOURCE           GPIO_PinSource7
#define L3GD20_SPI_MOSI_AF               GPIO_AF_SPI1

#define L3GD20_SPI_CS_PIN                GPIO_PIN_3                  /* PE.03 */
#define L3GD20_SPI_CS_GPIO_PORT          GPIOE                       /* GPIOE */
#define L3GD20_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOE

#define L3GD20_SPI_INT1_PIN              GPIO_PIN_0                  /* PE.00 */
#define L3GD20_SPI_INT1_GPIO_PORT        GPIOE                       /* GPIOE */
#define L3GD20_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define L3GD20_SPI_INT1_EXTI_LINE        EXTI_Line0
#define L3GD20_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define L3GD20_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
#define L3GD20_SPI_INT1_EXTI_IRQn        EXTI0_IRQn

#define L3GD20_SPI_INT2_PIN              GPIO_PIN_1                  /* PE.01 */
#define L3GD20_SPI_INT2_GPIO_PORT        GPIOE                       /* GPIOE */
#define L3GD20_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define L3GD20_SPI_INT2_EXTI_LINE        EXTI_Line1
#define L3GD20_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define L3GD20_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
#define L3GD20_SPI_INT2_EXTI_IRQn        EXTI1_IRQn

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_L3GD20		    ((uint8_t)0xD4)


#define L3GD20_CS_LOW()       HAL_GPIO_WritePin(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN,GPIO_PIN_RESET)
#define L3GD20_CS_HIGH()      HAL_GPIO_WritePin(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN,GPIO_PIN_SET)

/* Sensor Configuration Functions */
void L3GD20_SetSPI(SPI_HandleTypeDef h);
/*INT1 Interrupt Configuration Functions */
uint8_t L3GD20_GetDataStatus(void);
void L3GD20_Write(uint8_t buffer, uint8_t WriteAddr);
void L3GD20_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
/* Angular Data Functions */
float L3GD20_GetAngularRateX(float sensitivity);
float L3GD20_GetAngularRateY(float sensitivity);
float L3GD20_GetAngularRateZ(float sensitivity);

uint32_t L3GD20_TIMEOUT_UserCallback(void);

#endif /* L3GD20_H_ */
