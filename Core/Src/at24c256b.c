/*
 * at24c256b.c
 *
 *  Created on: 26.06.2023.
 *      Author: Artem Kagirov
 */

#include "at24c256b.h"

/*
 * @brief at24c256b_byte_write - Write byte to memory
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  pData Pointer to data buffer
 * @param  address of word in the EEPROM
 * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for STM32F3 family
 * @param GPIO_Pin_WP - pin on your MCU that is connected to WP pin AT24C256B
 */
void at24c256b_byte_write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t address,
	                      GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP)
{
	write_protection(0, GPIOx, GPIO_Pin_WP);
	uint8_t address_and_data[3] = {0};
	address_and_data[0] = (uint8_t)(address << 8);
	address_and_data[1] = (uint8_t)(address);
	address_and_data[2] = (uint8_t)*pData;
	HAL_I2C_Master_Transmit(hi2c, DevAddress, address_and_data, 3, 100);
}

/*
 * @brief at24c256b_page_write - Write page (64-byte) to memory
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  pData Pointer to data buffer
 * @param  Size Amount of data to be sent (after the EEPROM acknowledges
 *         receipt of the first data word, the microcontroller can transmit up to 63 more data words.
 *         The data word address lower six bits are internally incremented following the receipt of each
 *		   data word. The higher data word address bits are not incremented, retaining the memory page
 *		   row location. When the word address, internally generated, reaches the page boundary, the following
 *		   byte is placed at the beginning of the same page. If more than 64 data words are
 *		   transmitted to the EEPROM, the data word address will “roll over” and previous data will be
 *		   overwritten. The address “roll over” during write is from the last byte of the current page to the
 *		   first byte of the same page.)
 * @param  address of word in the EEPROM
 * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for STM32F3 family
 * @param GPIO_Pin_WP - pin on your MCU that is connected to WP pin AT24C256B
 */
void at24c256b_page_write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t Size, uint16_t address,
	                      GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP)
{
	write_protection(0, GPIOx, GPIO_Pin_WP);
	HAL_I2C_Mem_Write(hi2c, DevAddress, address, 2, pData, Size, 100);
}

/*
 * @brief at24c256b_byte_read - Random byte read
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  pData Pointer to data buffer
 * @param  address of word in the EEPROM
 * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for STM32F3 family
 * @param GPIO_Pin_WP - pin on your MCU that is connected to WP pin AT24C256B
 */
void at24c256b_byte_read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t address,
		                 GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP)
{
	write_protection(1, GPIOx, GPIO_Pin_WP);
	HAL_I2C_Mem_Read(hi2c, DevAddress, address, 2, (uint8_t * )pData, 1, 100);
}

/*
 * @brief at24c256b_sequential_read - Sequential byte reading
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  DevAddress Target device address: The device 7 bits address value
 *         in datasheet must be shifted to the left before calling the interface
 * @param  pData Pointer to data buffer
 * @param  Size Amount of data to be sent
 * @param  address of word in the EEPROM
 * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for STM32F3 family
 * @param GPIO_Pin_WP - pin on your MCU that is connected to WP pin AT24C256B
 */
void at24c256b_sequential_read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, int8_t* pData, uint16_t Size, uint16_t address,
		                 GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP)
{
	write_protection(1, GPIOx, GPIO_Pin_WP);
	HAL_StatusTypeDef result;
	result = HAL_I2C_Mem_Read(hi2c, DevAddress, address, 2, (uint8_t * )pData, Size, 100);
	__asm__ volatile("NOP");
}

/*
 * @brief  write_protection - Switch write protection
 * @param  wr_prot If wr_prot is 1 than write protection is ON else if wr_prot is 0 then
 *         write protection is OFF
 * @param  GPIOx where x can be (A..F) to select the GPIO peripheral for STM32F3 family
 * @param GPIO_Pin_WP - pin on your MCU that is connected to WP pin AT24C256B
 */
void write_protection(uint8_t wr_prot, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_WP)
{
	if(wr_prot == 1)
	{
		GPIOx->ODR |= GPIO_Pin_WP;
	}
	else if(wr_prot == 0)
	{
		GPIOx->ODR &= ~GPIO_Pin_WP;
	}
}
