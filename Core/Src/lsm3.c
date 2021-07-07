/*
 * lsm3.c
 *
 *  Created on: Jul 7, 2021
 *      Author: u
 */
#include "main.h"
#include "lsm3.h"
#include "spi.h"

#define _LIS3DHS_CS_ENBALE (CS_I2C_SPI_GPIO_Port->ODR &=~ CS_I2C_SPI_Pin)
#define _LIS3DHS_CS_DISABLE (CS_I2C_SPI_GPIO_Port->ODR |= CS_I2C_SPI_Pin)

void LIS3DSH_init(void)
    {
    u8 reg4 = LIS3DSH_DATARATE_25 | LIS3DSH_XYZ_ENABLE;
    u8 reg3 = LIS3DSH_INTERRUPT_SIGNAL_HIGH | LIS3DSH_INTERRUPT_1_ENABLE;
    u8 reg5 = LIS3DSH_FULLSCALE_4;

    LIS3DSH_Write(LIS3DSH_CTRL_REG5_ADDR, &reg5, 1);
    LIS3DSH_Write(LIS3DSH_CTRL_REG3_ADDR, &reg3, 1);
    LIS3DSH_Write(LIS3DSH_CTRL_REG4_ADDR, &reg4, 1);
    }

void LIS3DSH_GetDataRaw(LIS3DSH_DataRaw *data)
    {
    LIS3DSH_Read(LIS3DSH_OUT_X_L_ADDR, (u8*) &data->x, 6);
    }

void SPI_send(u8 *data, u8 len)
    {
    for (u8 i = 0; i < len; i++)
	{
	/* Wait until TXE flag is set to send data */
	while ((SPI1->SR & SPI_FLAG_TXE) == SPI_FLAG_TXE)
	    ;
	SPI1->DR = data[i];
	}
    }

void SPI_read(u8 *data, u8 len)
    {
    for (u8 i = 0; i < len; i++)
	{
	/* Check the RXNE flag */
	while ((SPI1->SR & SPI_FLAG_RXNE) == SPI_FLAG_RXNE)
	    ;
	/* read the received data */
	data[i] = SPI1->DR;
	}
    }

void SPI_wait(void)
    {
    while ((SPI1->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY)
	;
    }

void LIS3DSH_Write(u8 reg, u8 *data, u8 len)
    {
    //Enable CS
    _LIS3DHS_CS_ENBALE;
    //set register value
    HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
//    SPI_send(&reg, 1);
    //Transmit data
//    SPI_send(data, len);
    HAL_SPI_Transmit(&hspi1, data, len, 10);
//    SPI_wait();
    //Disable CS
    _LIS3DHS_CS_DISABLE;
    }

void LIS3DSH_Read(uint8_t reg, u8 *data, u8 len)
    {
    u8 reg_read = reg | 0x80;
    //Enable CS
    _LIS3DHS_CS_ENBALE;
    //set register value
    HAL_SPI_Transmit(&hspi1, &reg_read, 1, 10);
//    SPI_send(&reg_read, 1);
    //Transmit data
    HAL_SPI_Receive(&hspi1, data, len, 10);
//    SPI_read(data, len);
//    SPI_wait();
    //Disable CS
    _LIS3DHS_CS_DISABLE;
    }
