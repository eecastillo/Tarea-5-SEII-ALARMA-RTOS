# Tarea 5
## Eduardo Ethandrake Castillo Pulido ## Alejandro Gudiño Gallegos
## Funcionamiento General
El funcionamiento en general es, por medio de freeRTOS, generar una interrupcion cada segundo. En ese segundo se imprime la hora y se aumentan los segundos, y en caso de ser necesario el minuto y la hora. Para la implementacion se usan semaforos binarios y mutex, para hacer la comunicacion entre las diferentes funciones.


## Control de Segundos
Esto se genera por la funcion seconds_task, la cual se ejecuta de manera periodica cada segundo, en esa ejecucion se incrementa la variable seconds, la cual cuando es igual a 60, libera un semaforo y regresa a 0. Tambien, envia un mensaje llamado time_queue. Este mensaje se pasa por referencia.
```
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
```
## Control de Minutos
En esta funcion se espera a que se libere el semaforos minutes_semaphore. Cuando este semaforo se libera, incrementa una variable local minutes. Cuando la varialbe minutes llega a 60, se libera el semaforo hours_semaphores y regresa a 0. Tambien se hace set de la bandera de minutes en el event group y se envia un mensaje a time_queue por referencia.
```
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
```
## Control de horas
En esta funcion se espera a que se libere el semaforo hours_semaphore, cuando este se libera se incrementa una variable local hours. Cuando el valor de hours es igual a alarm.hours, se hace set de una bandera en el Event Group. Al llegar a 24, la variable regresa a 0.
```
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
```
## Activar Alarma
Se espera a que las tres banderas del event group esten activas, en caso de que las tres banderas esten activas se escribe ALARM en la terminal UART, luego se vuelve a esperar por el event group
```
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
```
## Imprimir hora
Para imprimir la hora se tiene una tarea que espera un mensaje por la queue, y cuando se llega ese nuevo mensaje, se actualiza el valor del tiempo local en horas, minutos y segundos. Cada que llega un mensaje nuevo, se imprime el valor de la hora actualizado por UART
```
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
```
