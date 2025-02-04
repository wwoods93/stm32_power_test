/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    aes.h
  * @brief   This file contains all the function prototypes for
  *          the aes.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AES_H__
#define __AES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CRYP_HandleTypeDef hcryp;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_AES_Init(void);
void MX_SAES_AES_Init(void);

/* USER CODE BEGIN Prototypes */
uint32_t hal_cryp_aes_get_word_of_key(uint8_t arg_index);
void hal_cryp_aes_generate_aes_key_from_rng(RNG_HandleTypeDef* arg_rng, uint32_t arg_random_uint32, uint8_t arg_is_random_number_generated);
void hal_cryp_aes_set_aes_key_word_at_index(uint32_t arg_word, uint8_t arg_index);
void hal_cryp_aes_write_aes_key_to_array(uint8_t* arg_array);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __AES_H__ */

