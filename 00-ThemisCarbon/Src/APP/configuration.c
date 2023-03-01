/*
 * configuration.c
 *
 *  Created on: 27 août 2017
 *      Author: Patrick
 */

#include "configuration.h"

float configuration_data[CONFIGURATION_DATA_COUNT];

volatile float * _configuration_data_flash_address = (float*)CONFIGURATION_FLASH_ADDR;

void HAL_Configuration_Init(void)
{
	///...
}

void HAL_Configuration_Reload(void)
{
	for(uint32_t index=0; index<CONFIGURATION_DATA_COUNT; ++index )
	{
		configuration_data[index] = _configuration_data_flash_address[index];
	}
}

void HAL_Configuration_Save(void)
{
	HAL_FLASH_Unlock();
#ifdef STM32F405xx
	FLASH_EraseInitTypeDef erase = { FLASH_TYPEERASE_SECTORS,FLASH_SECTOR_11, 1,FLASH_VOLTAGE_RANGE_3};  // Set the flash sector here according hal_flash_ex.h
	uint32_t fault_sec = 0;
	HAL_FLASHEx_Erase(&erase, &fault_sec);
	for(size_t index=0; index<CONFIGURATION_DATA_COUNT; ++index )
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(&_configuration_data_address[index]), _configuration_data[index]);
	}
#endif
#ifdef STM32L476xx
	FLASH_EraseInitTypeDef erase = {FLASH_TYPEERASE_PAGES,FLASH_BANK_1,CONFIGURATION_FLASH_PAGE,1};
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase,&error);
	for(size_t index=0; index<CONFIGURATION_DATA_COUNT; ++index )
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,(uint32_t)(&_configuration_data_address[index]),(uint64_t)_configuration_data[index]);
	}
#endif
#ifdef STM32F756xx
	FLASH_EraseInitTypeDef erase = {FLASH_TYPEERASE_SECTORS,FLASH_SECTOR_7,1,FLASH_VOLTAGE_RANGE_3};
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase,&error);
	for(size_t index=0; index<CONFIGURATION_DATA_COUNT; ++index )
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(&_configuration_data_flash_address[index]),*(uint32_t*)&configuration_data[index]);
	}
#endif
#ifdef STM32F767xx
	FLASH_EraseInitTypeDef erase = {FLASH_TYPEERASE_SECTORS,0,FLASH_SECTOR_7,1,FLASH_VOLTAGE_RANGE_3};
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase,&error);
	for(size_t index=0; index<CONFIGURATION_DATA_COUNT; ++index )
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(&_configuration_data_flash_address[index]),*(uint32_t*)&configuration_data[index]);
	}
#endif
	HAL_FLASH_Lock();
}
