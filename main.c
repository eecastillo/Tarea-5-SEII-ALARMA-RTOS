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
} alarm_t;

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


int main(void)
{
  alarm_t alarm;

  alarm.seconds = 0;
  alarm.minutes = 0;
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
  xTaskCreate(alarm_task, "Alarm", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);
  xTaskCreate(print_task, "Print", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);

  /* Force the counter to be placed into memory. */
  volatile static int i = 0 ;
  /* Enter an infinite loop, just incrementing a counter. */
  Seconds = xSemaphoreCreateBinary();
  Minutes = xSemaphoreCreateBinary();

  Queue = xQueueCreate(3, sizeof(time_msg_t));

  mutex = xSemaphoreCreateMutex();

  Group = xEventGroupCreate();

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
    PRINTF("ALARMA");
    xSemaphoreGive(mutex);
  }
}

void print_task(void *parameters)
{
  void* receiveData;
  time_msg_t message;
  for(;;)
  {
    xSemaphoreTake(mutex, portMAX_DELAY);
    PRINTF("Imprimir hora obtenida desde la Queue");
    xQueueReceive(Queue, receiveData, portMAX_DELAY);
    message = *((time_msg_t*)receiveData);
    switch (message.time_type) {
      case seconds_type:
      if(message.value < 10)
      {
        PRINTF("0");
      }
      PRINTF("%d", message.value);
      break;
      case minutes_type:
      if(message.value < 10)
      {
        PRINTF("0");
      }
      PRINTF("%d:", message.value);
      break;
      case hours_type:
        if(message.value < 10)
        {
          PRINTF("0");
        }
        PRINTF("%d:", message.value);
      break;
    }
    xSemaphoreGive(mutex);
  }
}


void second_task(void *parameters)
{
  time_msg_t seconds_msg;
  seconds_msg.time_type=seconds_type;
  uint8_t counter_seconds = 0;
  const EventBits_t secondsFlag = BIT_SEGUNDOS;
  TickType_t  xLastWakeTime = xTaskGetTickCount();
  TickType_t   xfactor = pdMS_TO_TICKS(1000);

  while(true)
  {
    vTaskDelayUntil(&xLastWakeTime,xfactor);
    seconds_msg.value = counter_seconds;
    xQueueSendToBack(Queue,(void*)seconds_msg, portMAX_DELAY);
    if(counter_seconds == alarm.seconds)
    {
      xEventGroupSetBits(Group, secondsFlag);
    }
    counter_seconds++;
    if(counter_seconds >= 60)
    {
      counter_seconds = 0;
      xSemaphoreGive(Seconds);
    }
  }
}
void minute_task(void *parameters)
{
  time_msg_t minutes_msg;
  minutes_msg.time_type=minutes_type;
  uint8_t counter_minutes = 0;
  const EventBits_t minutesFlag = BIT_MINUTOS;

  while(true)
  {
    xSemaphoreTake(Seconds, portMAX_DELAY);
    minutes_msg.value = counter_minutes;
    xQueueSendToBack(Queue,(void*)minutes_msg, portMAX_DELAY);
    if(counter_minutes == alarm.minutes)
    {
      xEventGroupSetBits(Group,minutesFlag);
    }
    counter_minutes++;
    if(counter_minutes == 60)
    {
      counter_minutes = 0;

      xSemaphoreGive(Minutes);
    }

  }
}
void hour_task(void *parameters)
{
  time_msg_t hours_msg;
  hours_msg.time_type=hours_type;
  uint8_t counter_hours = 0;
  const EventBits_t hoursFlag = BIT_HORAS;
  while(true)
  {
    xSemaphoreTake(Minutes, portMAX_DELAY);
    hours_msg.value = counter_hours;
    xQueueSendToBack(Queue,(void*)hours_msg, portMAX_DELAY);
    if(counter_hours == alarm.hours)
    {
      xEventGroupSetBits(Group,hoursFlag);
    }
    counter_hours++;

    if(counter_hours == 24)
    {
      counter_hours = 0;
    }

  }
}
