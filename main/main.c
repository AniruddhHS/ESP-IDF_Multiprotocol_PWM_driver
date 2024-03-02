#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "pwm_driver.h"

void app_main(void) {
	pwm_init();
	while (true) {
		for (int i = -90; i <= 90; i++) {
			pwm_write(0, i);
			pwm_write(1, i);
			pwm_write(2, i);
			pwm_write(3, i);

			printf("servo angle = %d\n", i);
			vTaskDelay(100);
		}

	}
}
