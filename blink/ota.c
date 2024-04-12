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

void task_ota_setup(void *pvParameters) {

	// int i;
	// TimerHandle_t oneshot_timer;

	// WiFi configuration
	struct sdk_station_config config = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, };
	sdk_wifi_station_set_auto_connect(1);
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();

	// OTA configuration
	ota_tftp_init_server(TFTP_PORT);

	// for (i = 0; i < interrupt_count_size; i++)
	// 	interrupt_count[i] = 0;

	// // configure GPIO pins
	// gpio_enable(gpio_interrupt12, GPIO_INPUT);
	// gpio_enable(gpio_interrupt13, GPIO_INPUT);
	// gpio_enable(gpio_frc1, GPIO_OUTPUT);
	// gpio_enable(gpio_led, GPIO_INPUT);

	// // set initial values of interrupt pins
	// gpio_write(gpio_interrupt12, 1);
	// gpio_write(gpio_interrupt13, 1);

	// // stop frc1 timer and mask interrupts as a precaution
	// timer_set_interrupts(FRC1, false);
	// timer_set_run(FRC1, false);

	// // set up ISRs
	// _xt_isr_attach(INUM_TIMER_FRC1, frc1_interrupt_handler, NULL);
	// gpio_set_interrupt(gpio_interrupt12, GPIO_INTTYPE_EDGE_NEG, gpio_intr_handler);
	// gpio_set_interrupt(gpio_interrupt13, GPIO_INTTYPE_EDGE_NEG, gpio_intr_handler);

	// // configure timer frequency
	// timer_set_frequency(FRC1, freq_frc1);

	// // unmask interrupts and start frc1 timer
	// timer_set_interrupts(FRC1, true);
	// timer_set_run(FRC1, true);

	// // create interrupt handling task
	// xTaskCreate(interrupt_handling_task, "Interrupt handling task", 1000, NULL, 2, &interrupt_task_handle);

	// // create periodic task
	// xTaskCreate(periodic_task, "Periodic task", 1000, NULL, 1, NULL);

	// // set up one-shot timer
	// oneshot_timer = xTimerCreate("One-shot timer", pdMS_TO_TICKS(15000), pdFALSE, NULL, oneshot_handler);
	// xTimerStart(oneshot_timer, 0);
}
