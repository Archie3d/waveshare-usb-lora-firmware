#include <FreeRTOS.h>
#include <task.h>

#include "init.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    for (;;) {
        __asm__("nop");
    }
}

int main(void)
{
    global_init();

    for(;;);
    return 0;
}
