/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    otfdec.c
  * @brief   This file provides code for the configuration
  *          of the OTFDEC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "otfdec.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

OTFDEC_HandleTypeDef hotfdec1;
OTFDEC_HandleTypeDef hotfdec2;

/* OTFDEC1 init function */
void MX_OTFDEC1_Init(void)
{

  /* USER CODE BEGIN OTFDEC1_Init 0 */

  /* USER CODE END OTFDEC1_Init 0 */

  /* USER CODE BEGIN OTFDEC1_Init 1 */

  /* USER CODE END OTFDEC1_Init 1 */
  hotfdec1.Instance = OTFDEC1;
  if (HAL_OTFDEC_Init(&hotfdec1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OTFDEC1_Init 2 */

  /* USER CODE END OTFDEC1_Init 2 */

}
/* OTFDEC2 init function */
void MX_OTFDEC2_Init(void)
{

  /* USER CODE BEGIN OTFDEC2_Init 0 */

  /* USER CODE END OTFDEC2_Init 0 */

  /* USER CODE BEGIN OTFDEC2_Init 1 */

  /* USER CODE END OTFDEC2_Init 1 */
  hotfdec2.Instance = OTFDEC2;
  if (HAL_OTFDEC_Init(&hotfdec2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OTFDEC2_Init 2 */

  /* USER CODE END OTFDEC2_Init 2 */

}

void HAL_OTFDEC_MspInit(OTFDEC_HandleTypeDef* otfdecHandle)
{

  if(otfdecHandle->Instance==OTFDEC1)
  {
  /* USER CODE BEGIN OTFDEC1_MspInit 0 */

  /* USER CODE END OTFDEC1_MspInit 0 */
    /* OTFDEC1 clock enable */
    __HAL_RCC_OTFDEC1_CLK_ENABLE();

    /* OTFDEC1 interrupt Init */
    HAL_NVIC_SetPriority(OTFDEC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTFDEC1_IRQn);
  /* USER CODE BEGIN OTFDEC1_MspInit 1 */

  /* USER CODE END OTFDEC1_MspInit 1 */
  }
  else if(otfdecHandle->Instance==OTFDEC2)
  {
  /* USER CODE BEGIN OTFDEC2_MspInit 0 */

  /* USER CODE END OTFDEC2_MspInit 0 */
    /* OTFDEC2 clock enable */
    __HAL_RCC_OTFDEC2_CLK_ENABLE();

    /* OTFDEC2 interrupt Init */
    HAL_NVIC_SetPriority(OTFDEC2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTFDEC2_IRQn);
  /* USER CODE BEGIN OTFDEC2_MspInit 1 */

  /* USER CODE END OTFDEC2_MspInit 1 */
  }
}

void HAL_OTFDEC_MspDeInit(OTFDEC_HandleTypeDef* otfdecHandle)
{

  if(otfdecHandle->Instance==OTFDEC1)
  {
  /* USER CODE BEGIN OTFDEC1_MspDeInit 0 */

  /* USER CODE END OTFDEC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_OTFDEC1_CLK_DISABLE();

    /* OTFDEC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(OTFDEC1_IRQn);
  /* USER CODE BEGIN OTFDEC1_MspDeInit 1 */

  /* USER CODE END OTFDEC1_MspDeInit 1 */
  }
  else if(otfdecHandle->Instance==OTFDEC2)
  {
  /* USER CODE BEGIN OTFDEC2_MspDeInit 0 */

  /* USER CODE END OTFDEC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_OTFDEC2_CLK_DISABLE();

    /* OTFDEC2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(OTFDEC2_IRQn);
  /* USER CODE BEGIN OTFDEC2_MspDeInit 1 */

  /* USER CODE END OTFDEC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
