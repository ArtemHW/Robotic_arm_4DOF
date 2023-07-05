/*
 * at24c256b.h
 *
 *  Created on: 26.06.2023.
 *      Author: Artem Kagirov
 */

#ifndef INC_AT24C256B_H_
#define INC_AT24C256B_H_

#include "stm32f3xx_hal.h"

void at24c256b_byte_write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t address,
	                      GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP);
void at24c256b_page_write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t Size, uint16_t address,
	                      GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP);
void at24c256b_byte_read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t address,
		                 GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP);
void at24c256b_sequential_read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t Size, uint16_t address,
		                 GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP);
void write_protection(uint8_t wr_prot, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP);

#endif /* INC_AT24C256B_H_ */
