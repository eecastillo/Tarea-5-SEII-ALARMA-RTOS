/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    alarm.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* TODO: insert other definitions and declarations here. */

#define BIT_SEGUNDOS (1 << 0)
#define BIT_MINUTOS (1 << 1)
#define BIT_HORAS (1 << 2)
#define ZERO_NOT 10u

typedef enum
{
  seconds_type,
  minutes_type,
  hours_type
} time_types_t;

typedef struct
{
	time_types_t time_type;
	uint8_t value;
} time_msg_t;

typedef struct
{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
}clock_time_t;

void second_task(void *parameters);
void minute_task(void *parameters);
void hour_task(void *parameters);

void alarm_task(void *parameters);
void print_task(void *parameters);

SemaphoreHandle_t Seconds;
SemaphoreHandle_t Minutes;

QueueHandle_t Queue;

EventGroupHandle_t Group;

SemaphoreHandle_t mutex;

clock_time_t alarm;

int main(void)
{

  alarm.seconds = 5;
  alarm.minutes = 1;
  alarm.hours = 0;

  /* Init board hardware. */
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
  /* Init FSL debug console. */
  BOARD_InitDebugConsole();
#endif

  xTaskCreate(second_task, "Seconds", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);
  xTaskCreate(minute_task, "Minutes", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);
  xTaskCreate(hour_task, "Hours", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);
  xTaskCreate(alarm_task, "Alarm", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL);
  xTaskCreate(print_task, "Print", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);

  /* Force the counter to be placed into memory. */
  volatile static int i = 0 ;
  /* Enter an infinite loop, just incrementing a counter. */
  Seconds = xSemaphoreCreateBinary();
  Minutes = xSemaphoreCreateBinary();

  Queue = xQueueCreate(3, sizeof(time_msg_t));

  mutex = xSemaphoreCreateMutex();

  Group = xEventGroupCreate();
  PRINTF("Configurado\n");
  vTaskStartScheduler();
  while(1) {
      i++ ;
      /* 'Dummy' NOP to allow source level single stepping of
          tight while() loop */
      __asm volatile ("nop");
  }
  return 0 ;
}

void alarm_task(void *parameters)
{
  const EventBits_t xBitsToWaitFor = ( BIT_SEGUNDOS | BIT_MINUTOS | BIT_HORAS );
  EventBits_t xEventGroupValue;

  for(;;)
  {

    xEventGroupValue = xEventGroupWaitBits(Group, xBitsToWaitFor, pdTRUE, pdTRUE, portMAX_DELAY);

    xSemaphoreTake(mutex, portMAX_DELAY);
    PRINTF("\nALARMA\n");
    xSemaphoreGive(mutex);
  }
}

void print_task(void *parameters)
{
  clock_time_t clock_time={0,0,0};
  time_msg_t message;
  portBASE_TYPE xStatus;

  for(;;)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    xQueueReceive(Queue, &message, portMAX_DELAY);
    switch(message.time_type)
    {
    	case seconds_type:
    		clock_time.seconds = message.value;
    		break;
    	case minutes_type:
    		clock_time.minutes = message.value;
    		break;
    	case hours_type:
    		clock_time.hours = message.value;
    		break;
    }

    //PRINTF("\e[1;1H\e[2J");
      if(clock_time.hours < ZERO_NOT)
      {
    	  PRINTF("\r0");
      }
      PRINTF("%d:",clock_time.hours);
      if(clock_time.minutes < ZERO_NOT)
      {
    	  PRINTF("0");
      }
      PRINTF("%d:",clock_time.minutes);
      if(clock_time.seconds < ZERO_NOT)
      {
    	  PRINTF("0");
      }
      PRINTF("%d",clock_time.seconds);
    xSemaphoreGive(mutex);
  }
}


void second_task(void *parameters)
{
  time_msg_t seconds_msg;
  seconds_msg.time_type=seconds_type;
  uint8_t counter_seconds = 0;
  TickType_t  xLastWakeTime = xTaskGetTickCount();
  TickType_t   xfactor = pdMS_TO_TICKS(1000);
  portBASE_TYPE xStatus;

  while(true)
  {
	if(counter_seconds == alarm.seconds)
	{
		xEventGroupSetBits(Group, BIT_SEGUNDOS);
	}
	else
	{
		xEventGroupClearBits(Group, BIT_SEGUNDOS);
	}
    vTaskDelayUntil(&xLastWakeTime,xfactor);
   // PRINTF("segundos\n");
    counter_seconds++;
    if(counter_seconds >= 60)
    {
      counter_seconds = 0;
      xSemaphoreGive(Seconds);
    }
    seconds_msg.value = counter_seconds;
    xStatus = xQueueSendToBack(Queue,&seconds_msg, 0);
    if (xStatus != pdPASS)
	{
	  /* We could not write to the queue because it was full � this must
	   be an error as the queue should never contain more than one item! */
	  PRINTF("Could not send to the queue.\r\n");
	}
  }
}
void minute_task(void *parameters)
{
  time_msg_t minutes_msg;
  minutes_msg.time_type=minutes_type;
  uint8_t counter_minutes = 0;
  portBASE_TYPE xStatus;

  while(true)
  {
	if(counter_minutes == alarm.minutes)
	{
	  xEventGroupSetBits(Group,BIT_MINUTOS);
	}
	else
	{
		xEventGroupClearBits(Group, BIT_MINUTOS);
	}
    xSemaphoreTake(Seconds, portMAX_DELAY);
    counter_minutes++;
    if(counter_minutes == 60)
    {
      counter_minutes = 0;
      xSemaphoreGive(Minutes);
    }
    minutes_msg.value = counter_minutes;
    xStatus = xQueueSendToBack(Queue,&minutes_msg, portMAX_DELAY);
    if (xStatus != pdPASS)
	{
	  /* We could not write to the queue because it was full � this must
	   be an error as the queue should never contain more than one item! */
	  PRINTF("Could not send to the queue.\r\n");
	}
  }
}
void hour_task(void *parameters)
{
  time_msg_t hours_msg;
  hours_msg.time_type=hours_type;
  uint8_t counter_hours = 0;
  portBASE_TYPE xStatus;

  while(true)
  {
    if(counter_hours == alarm.hours)
    {
	  xEventGroupSetBits(Group,BIT_HORAS);
    }
	else
	{
		xEventGroupClearBits(Group, BIT_HORAS);
	}
    xSemaphoreTake(Minutes, portMAX_DELAY);
    counter_hours++;
    if(counter_hours == 24)
    {
      counter_hours = 0;
    }
    hours_msg.value = counter_hours;
    xStatus = xQueueSendToBack(Queue,&hours_msg, portMAX_DELAY);
    if (xStatus != pdPASS)
	{
	  /* We could not write to the queue because it was full � this must
	   be an error as the queue should never contain more than one item! */
	  PRINTF("Could not send to the queue.\r\n");
	}
  }
}
