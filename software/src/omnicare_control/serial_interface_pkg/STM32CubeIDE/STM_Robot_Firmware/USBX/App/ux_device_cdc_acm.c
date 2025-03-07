/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device CDC ACM applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <inttypes.h>

#include "robot_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;
extern UART_HandleTypeDef huart1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  /* Save the CDC instance */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  /* Reset the cdc acm instance */
  cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
	VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
	{
		/* Private Variables */
		ULONG rx_actual_length;

		/* Infinite Loop */
		while(1)
		{
		   if(cdc_acm != UX_NULL)
		   {
			   char velocity_pwm_rx[20] = {0};
			   ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)velocity_pwm_rx, sizeof(velocity_pwm_rx), &rx_actual_length);

			   if (rx_actual_length <= sizeof(velocity_pwm_rx) ) {
				   if(velocity_pwm_rx[0]=='m') set_motors_velocity_string(velocity_pwm_rx);
			   }
		   }
		}
	}


/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */

char encoder_message[40];

VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input)
{
    /* Private Variables */
    ULONG tx_actual_length;
    while(1)
    {
    	char encoder_message[40];

    	sprintf(encoder_message, "%" PRId32 " %" PRId32 " %" PRId32 "\n",
    			motors_data[0].encoder,
				motors_data[1].encoder,
				motors_data[2].encoder);
//    	sprintf(encoder_message, "100 200 300\n");



//    	HAL_UART_Transmit(&huart1,(uint8_t *)"I will read the USB-C!\n\r", sizeof("I will read the USB-C!\n\r"), 1000);
		ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)(encoder_message), sizeof(encoder_message), &tx_actual_length);

		tx_thread_sleep(10);
    }

}

/* USER CODE END 1 */
