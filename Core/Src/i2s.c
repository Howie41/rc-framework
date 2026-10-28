/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2s.c
  * @brief   This file provides code for the configuration
  *          of the I2S instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2s.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2S_HandleTypeDef hi2s6;

/* I2S6 init function */
void MX_I2S6_Init(void)
{

  /* USER CODE BEGIN I2S6_Init 0 */

  /* USER CODE END I2S6_Init 0 */

  /* USER CODE BEGIN I2S6_Init 1 */

  /* USER CODE END I2S6_Init 1 */
  hi2s6.Instance = SPI6;
  hi2s6.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s6.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s6.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s6.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s6.Init.AudioFreq = I2S_AUDIOFREQ_11K;
  hi2s6.Init.CPOL = I2S_CPOL_LOW;
  hi2s6.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s6.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s6.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s6.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S6_Init 2 */

  /* USER CODE END I2S6_Init 2 */

}

void HAL_I2S_MspInit(I2S_HandleTypeDef* i2sHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2sHandle->Instance==SPI6)
  {
  /* USER CODE BEGIN SPI6_MspInit 0 */

  /* USER CODE END SPI6_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* I2S6 clock enable */
    __HAL_RCC_SPI6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**I2S6 GPIO Configuration
    PG8     ------> I2S6_WS
    PG13     ------> I2S6_CK
    PG14     ------> I2S6_SDO
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI6_MspInit 1 */

  /* USER CODE END SPI6_MspInit 1 */
  }
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef* i2sHandle)
{

  if(i2sHandle->Instance==SPI6)
  {
  /* USER CODE BEGIN SPI6_MspDeInit 0 */

  /* USER CODE END SPI6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI6_CLK_DISABLE();

    /**I2S6 GPIO Configuration
    PG8     ------> I2S6_WS
    PG13     ------> I2S6_CK
    PG14     ------> I2S6_SDO
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_8|GPIO_PIN_13|GPIO_PIN_14);

  /* USER CODE BEGIN SPI6_MspDeInit 1 */

  /* USER CODE END SPI6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

