#ifdef ENABLE_OTA
#ifdef ENABLE_WIFI

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "esp8266.h"
#include "config.h"
#include "ota-tftp.h"

const int gpio_frc1 = 14;			// D5
const int gpio_interrupt12 = 12;	// D6
const int gpio_interrupt13 = 13;	// D7
const int gpio_led = 2;				// D4

const int max_pin_no = 14;
const int freq_frc1 = 1;

#define interrupt_count_size	3
// interrupt counter for each type of interrupt
// interrupt_count[0]: counter for max_pin_no ...
static volatile uint32_t interrupt_count[interrupt_count_size];
static TaskHandle_t interrupt_task_handle;
static uint32_t pin_num;

void autoreload_handler(TimerHandle_t xTimer) {

	int i;

	printf("Running auto-reload timer.\n");
	for (i = 0; i < interrupt_count_size; i++)
		printf("interrupt counter %d: %d\n", i, interrupt_count[i]);
}

void oneshot_handler(TimerHandle_t xTimer) {

	TimerHandle_t autoreload_timer;

	printf("Creating auto-reload timer.\n");
	autoreload_timer = xTimerCreate("Auto-reload timer", pdMS_TO_TICKS(10000), pdTRUE, NULL, autoreload_handler);
	xTimerStart(autoreload_timer, 0);
}

void periodic_task(void *pvParameter) {

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(5000));
		printf("Periodic task unblocked.\n");
	}
}

void interrupt_handling_task(void *pvParameter) {

	BaseType_t notification_received;
	uint32_t notification;

	while (1) {
		notification_received = xTaskNotifyWait(pdFALSE, 0xffffffffUL, &notification, portMAX_DELAY);
		if (notification_received == pdPASS) {
			printf("Interrupt occurred: pin %d\n", notification);
		}
	}
}

void gpio_intr_handler(uint8_t gpio_num) {

	static TickType_t last_interrupt_time = 0;
	TickType_t current_time;
	BaseType_t wake_higher_task = pdFALSE;

	current_time = xTaskGetTickCountFromISR();
	if (current_time - last_interrupt_time > pdMS_TO_TICKS(200)) {

		last_interrupt_time = current_time;

		// increment interrupt counter
		interrupt_count[max_pin_no - gpio_num]++;

		pin_num = gpio_num;
		xTaskNotifyFromISR(interrupt_task_handle, pin_num, eSetValueWithOverwrite, &wake_higher_task);
		portEND_SWITCHING_ISR(wake_higher_task);
	}
	last_interrupt_time = current_time;
}

void frc1_interrupt_handler(void *arg) {

	BaseType_t wake_higher_task = pdFALSE;

	// increment interrupt counter
	interrupt_count[max_pin_no - gpio_frc1]++;

	gpio_toggle(gpio_frc1);

	pin_num = gpio_frc1;
	xTaskNotifyFromISR(interrupt_task_handle, pin_num, eSetValueWithOverwrite, &wake_higher_task);
	portEND_SWITCHING_ISR(wake_higher_task);
}

#endif
#endif
