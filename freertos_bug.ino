#include <FreeRTOS.h>
#include <queue.h>
#include <cstdint>
#include <sam.h>
#include "SAMD51_TC.h"

static TaskHandle_t task = nullptr;

static QueueHandle_t  queue = nullptr;

// ISR handler called when the TC3 period has elapsed.
static void isrCallback()
{
    // Post counter value to the loop task.
    static std::uint8_t value = 0;
    xQueueSendFromISR(queue, &value, nullptr);
    value += 1;
}

void loop();
static void loopTask(void*)
{
    for(;;) {
        loop();
        vTaskDelay(0);
    }
}

void setup() {
    queue = xQueueCreate( 32, 1 );
    configASSERT(queue);

    TimerTC3.initialize();
    TimerTC3.setPriority(3);
    // Uncomment if you want to reproduce the Seeed_Arduino_FreeRTOS priority issue.
    //TimerTC3.setPriority(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    TimerTC3.attachInterrupt(isrCallback);

    Serial.begin(115200);
    Serial.println("start");

    xTaskCreate(loopTask, "loop", 2048, nullptr, 0, &task);
    vTaskStartScheduler();
}
void loop() {
    std::uint8_t value;
    // Receive the value sent from the TC3 ISR.
    xQueueReceive(queue, &value, portMAX_DELAY);
    Serial.println(value);
}
