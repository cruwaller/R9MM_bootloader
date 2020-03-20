/**
 * @file    flash.c
 * @author  Ferenc Nemeth
 * @date    21 Dec 2018
 * @brief   This module handles the memory related functions.
 *
 *          Copyright (c) 2018 Ferenc Nemeth - https://github.com/ferenc-nemeth
 */

#include "flash.h"

/* Function pointer for jumping to user application. */
typedef void (*fnc_ptr)(void);

/**
 * @brief   This function erases the memory.
 * @param   address: First address to be erased (the last is the end of the
 * flash).
 * @return  status: Report about the success of the erasing.
 */
flash_status flash_erase(uint32_t address) {
  HAL_FLASH_Unlock();

  flash_status status = FLASH_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = address;
  erase_init.Banks = FLASH_BANK_1;
  /* Calculate the number of pages from "address" and the end of flash. */
  erase_init.NbPages = (FLASH_BANK1_END - address) / FLASH_PAGE_SIZE;
  /* Do the actual erasing. */
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error)) {
    status = FLASH_OK;
  }

  HAL_FLASH_Lock();

  return status;
}

/**
 * @brief   This function flashes the memory.
 * @param   address: First address to be written to.
 * @param   *data:   Array of the data that we want to write.
 * @param   *length: Size of the array.
 * @return  status: Report about the success of the writing.
 */
flash_status flash_write(uint32_t address, uint32_t *data, uint32_t length) {
  flash_status status = FLASH_OK;

  HAL_FLASH_Unlock();

  /* Loop through the array. */
  for (uint32_t i = 0u; (i < length) && (FLASH_OK == status); i++) {
    /* If we reached the end of the memory, then report an error and don't do
     * anything else.*/
    if (FLASH_APP_END_ADDRESS <= address) {
      status |= FLASH_ERROR_SIZE;
    } else {
      /* The actual flashing. If there is an error, then report it. */
      if (HAL_OK !=
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i])) {
        status |= FLASH_ERROR_WRITE;
      }
      /* Read back the content of the memory. If it is wrong, then report an
       * error. */
      if (((data[i])) != (*(volatile uint32_t *)address)) {
        status |= FLASH_ERROR_READBACK;
      }

      /* Shift the address by a word. */
      address += 4u;
    }
  }

  HAL_FLASH_Lock();

  return status;
}

/**
 * @brief   This function flashes the memory.
 * @param   address: First address to be written to.
 * @param   *data:   Array of the data that we want to write.
 * @param   *length: Size of the array.
 * @return  status: Report about the success of the writing.
 */
flash_status flash_write_halfword(uint32_t address, uint16_t *data,
                                  uint32_t length) {
  flash_status status = FLASH_OK;

  HAL_FLASH_Unlock();

  /* Loop through the array. */
  for (uint32_t i = 0u; (i < length) && (FLASH_OK == status); i++) {
    /* If we reached the end of the memory, then report an error and don't do
     * anything else.*/
    if (FLASH_APP_END_ADDRESS <= address) {
      status |= FLASH_ERROR_SIZE;
    } else {
      /* The actual flashing. If there is an error, then report it. */
      if (HAL_OK !=
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data[i])) {
        status |= FLASH_ERROR_WRITE;
      }
      /* Read back the content of the memory. If it is wrong, then report an
       * error. */
      if (((data[i])) != (*(volatile uint16_t *)address)) {
        status |= FLASH_ERROR_READBACK;
      }

      /* Shift the address by a word. */
      address += 2u;
    }
  }

  HAL_FLASH_Lock();

  return status;
}

#if 0
/*******************************************************************************
 * Function Name  : FLASH_ProgramHalfWord
 * Description    : Programs a half word at a specified address.
 * Input          : - Address: specifies the address to be programmed.
 *                  - Data: specifies the data to be programmed.
 * Output         : None
 * Return         : FLASH Status: The returned value can be: FLASH_BUSY,
 *                  FLASH_ERROR_PG or FLASH_ERROR_WRP or FLASH_COMPLETE or
 *                  FLASH_TIMEOUT.
 *******************************************************************************/
flash_status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data) {
  flash_status status = FLASH_OK;

  /* Check the parameters */
  //  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(ProgramTimeout);

  if (status == FLASH_COMPLETE) {
    /* if the previous operation is completed, proceed to program the new data
     */
    FLASH->CR |= CR_PG_Set;

    *(vu16 *)Address = Data;
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if (status != FLASH_BUSY) {
      /* if the program operation is completed, disable the PG Bit */
      FLASH->CR &= CR_PG_Reset;
    }
  }
  /* Return the Program Status */
  return status;
}

/*******************************************************************************
 * Function Name  : FLASH_ProgramOptionByteData
 * Description    : Programs a half word at a specified Option Byte Data
 *address. Input          : - Address: specifies the address to be programmed.
 *                    This parameter can be 0x1FFFF804 or 0x1FFFF806.
 *                  - Data: specifies the data to be programmed.
 * Output         : None
 * Return         : FLASH Status: The returned value can be: FLASH_BUSY,
 *                  FLASH_ERROR_PG or FLASH_ERROR_WRP or FLASH_COMPLETE or
 *                  FLASH_TIMEOUT.
 *******************************************************************************/
flash_status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data) {
  flash_status status = FLASH_OK;

  /* Check the parameters */
  //  assert_param(IS_OB_DATA_ADDRESS(Address));

  status = FLASH_WaitForLastOperation(ProgramTimeout);

  if (status == FLASH_COMPLETE) {
    /* Authorize the small information block programming */
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;

    /* Enables the Option Bytes Programming operation */
    FLASH->CR |= CR_OPTPG_Set;
    *(vu16 *)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if (status != FLASH_BUSY) {
      /* if the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= CR_OPTPG_Reset;
    }
  }
  /* Return the Option Byte Data Program Status */
  return status;
}
#endif

/**
 * @brief   Actually jumps to the user application.
 * @param   void
 * @return  void
 */
void flash_jump_to_app(void) {
  /* Function pointer to the address of the user application. */
  fnc_ptr jump_to_app;
  jump_to_app = (fnc_ptr)(*(volatile uint32_t *)(FLASH_APP_START_ADDRESS + 4u));
  HAL_DeInit();
  /* Change the main stack pointer. */
  __set_MSP(*(volatile uint32_t *)FLASH_APP_START_ADDRESS);
  jump_to_app();
}
