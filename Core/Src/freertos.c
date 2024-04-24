/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "usart.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f1xx_hal.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t xQueueUSARTtoSPI;
QueueHandle_t xQueueSPItoUSART;
SemaphoreHandle_t xMutex;

uint8_t filler_data[BUFFER_SIZE];
uint8_t received_data[BUFFER_SIZE];
uint8_t uartBufferRX[BUFFER_SIZE];
uint8_t uartBufferTX[BUFFER_SIZE];
uint8_t tmpBuff[BUFFER_SIZE];

bool fUartToSpiSending = false;
UBaseType_t sizeOfQueue = 10;

/* USER CODE END Variables */
/* Definitions for TaskSPI */
osThreadId_t TaskSPIHandle;
const osThreadAttr_t TaskSPI_attributes = {
  .name = "TaskSPI",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskUSART */
osThreadId_t TaskUSARTHandle;
const osThreadAttr_t TaskUSART_attributes = {
  .name = "TaskUSART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vTaskUSART(void *pvParameters);
void vTaskSPI(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void vTaskSPI(void *argument);
void vTaskUSART(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	xMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    xQueueUSARTtoSPI = xQueueCreate(sizeOfQueue, BUFFER_SIZE);
    xQueueSPItoUSART = xQueueCreate(sizeOfQueue, BUFFER_SIZE);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskSPI */
  TaskSPIHandle = osThreadNew(vTaskSPI, NULL, &TaskSPI_attributes);

  /* creation of TaskUSART */
  TaskUSARTHandle = osThreadNew(vTaskUSART, NULL, &TaskUSART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_vTaskSPI */
/**
  * @brief  Function implementing the TaskSPI thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vTaskSPI */
void vTaskSPI(void *argument)
{
  /* USER CODE BEGIN vTaskSPI */

	memset(&filler_data[0], 0, BUFFER_SIZE);
	memset(&received_data[0], 0, BUFFER_SIZE);

	fSpi1EndTransmission = false;

	HAL_SPI_Transmit_DMA(&hspi1, filler_data, BUFFER_SIZE); // не цекличное
	HAL_SPI_Receive_DMA(&hspi1, received_data, BUFFER_SIZE);


  /* Infinite loop */
    for(;;)
    {
    	do
    	{
    	    if(fSpi1EndTransmission)
    	    {
    	    	uint16_t packedSize = 0;
    		    // ======================== Outgoing Packet Handler =============================
    		    if(fUartToSpiSending || uxQueueMessagesWaiting(xQueueUSARTtoSPI) > 0)
    		    {
    	    		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    	    		{
        		        if (xQueueReceive(xQueueUSARTtoSPI, (void*)tmpBuff, 0) == pdTRUE)
        		        {
        		    	     memset(&filler_data[0], 0, BUFFER_SIZE);
        		    	     packedSize = strlen((const char *)tmpBuff);
        		    	     memcpy(&filler_data[0], &tmpBuff[0], packedSize);

        		        }
        		        else
        		        {
        		        	xSemaphoreGive(xMutex);
        			        break;
        		        }
        		        xSemaphoreGive(xMutex);
    	    		}
    		    }
    		    else
    		    {
    		    	//If the queue for sending is full, then we will wait
    		    	UBaseType_t count = uxQueueMessagesWaiting(xQueueSPItoUSART);
    		    	if(count < sizeOfQueue)
    		    	{
    		    		memset(&filler_data[0], 0, BUFFER_SIZE);
    		    		packedSize = BUFFER_SIZE;
    		    	}
    		    	else
    		    		break;
    		    }


    		    // ======================== Incoming Packet Handler =============================
    		    if (xQueueSend(xQueueUSARTtoSPI, (void*)received_data, portMAX_DELAY) != pdPASS)
    		    {
    		        // Error sending data to queue
    		    }
    		    memset(&received_data[0], 0, BUFFER_SIZE);

    		    fSpi1EndTransmission = false;
    		    HAL_SPI_Transmit_DMA(&hspi1, filler_data, packedSize);
    		    HAL_SPI_Receive_DMA(&hspi1, received_data, BUFFER_SIZE);
    	    }
    	}
    	while(0);



      osDelay(10);
    }
  /* USER CODE END vTaskSPI */
}

/* USER CODE BEGIN Header_vTaskUSART */
/**
* @brief Function implementing the TaskUSART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskUSART */
void vTaskUSART(void *argument)
{
  /* USER CODE BEGIN vTaskUSART */
	uint16_t currentPos = 0;
	memset(&uartBufferRX[0], 0, BUFFER_SIZE);
	memset(&uartBufferTX[0], 0, BUFFER_SIZE);
	HAL_UART_Receive_DMA(&huart1, &uartBufferRX[0], BUFFER_SIZE); // circle

	uint32_t delay_ms = calculate_uart_delay(huart1.Init.BaudRate, (uint32_t)BUFFER_SIZE);
	uint32_t delay_counter = delay_ms;
	fUartToSpiSending = false;
  /* Infinite loop */
    for(;;)
    {
    	// ======================== Incoming Packet Handler =============================
    	if(--delay_counter == 0)
    	{
    		delay_counter = delay_ms;


    	    uint16_t newPos = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    	    if(currentPos != newPos)
    	    {


    	    	uint16_t len = currentPos > newPos ? BUFFER_SIZE - currentPos + newPos : newPos - currentPos;
     	    	uint16_t tmpPos = currentPos;
    		    uint32_t i = 0;
    		    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    		    {
    		    	fUartToSpiSending = true;
    		        memset(&tmpBuff[0], 0, BUFFER_SIZE);
    		        do
                    {
    			        tmpPos += 1;
    			        tmpPos %= BUFFER_SIZE;
    			        tmpBuff[i++] = uartBufferRX[tmpPos];

            	        if(uartBufferRX[tmpPos] == 0x00)
            	        {
            	    	    fUartToSpiSending = false;
            	    	    if(len == 1) break;
            	        }

                    }
    		        while(--len > 0);

    		        if(len > 0)
    			        currentPos = tmpPos;
    		        else
    			        currentPos = newPos;

                    if (tmpBuff[0]!= 0 && xQueueSend(xQueueUSARTtoSPI, (void*)tmpBuff, portMAX_DELAY) == pdPASS)
                    {
                        //
                    }
                    xSemaphoreGive(xMutex);
    		    }
    	    }
    	}

    	// ======================== Outgoing Packet Handler =============================
    	if (huart1.gState != HAL_UART_STATE_READY)
    	{
    		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    		{
  		        if (xQueueReceive(xQueueSPItoUSART, (void*)tmpBuff, 0) == pdTRUE)
   		        {
  		    	    uint16_t packedSize = strlen((const char *)tmpBuff) + 1;
  		    	    memcpy(&uartBufferTX[0], &tmpBuff[0], packedSize);

  		    	    if (HAL_UART_Transmit_DMA(&huart1, &uartBufferTX[0], packedSize) != HAL_OK)
   			        {
   			           // Error in transmission
   			           //Error_Handler();
   			        }
   		        }
  		        xSemaphoreGive(xMutex);
   		    }
    	}

        osDelay(1);
    }
  /* USER CODE END vTaskUSART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

