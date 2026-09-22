#include "startup_task.h"

#include "cmsis_os.h"
#include "lwip.h"
#include "main.h"

void StartStartupTask(void *argument)
{
  (void)argument;

  MX_LWIP_Init();

  for (;;)
  {
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    osDelay(500);
  }
}
