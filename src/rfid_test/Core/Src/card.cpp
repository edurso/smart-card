#include "card.hpp"

#include "main.h"

// Handles defined in IOC
extern TIM_HandleTypeDef htim6; // 1 MHz
extern TIM_HandleTypeDef htim7; // 1 Hz

// Rename handles
#define BLINK_TIMER &htim7

namespace card {

	GPIOPin light;

	auto init() -> void {
		light = GPIOPin(GPIOB, GPIO_PIN_3);
		if (HAL_TIM_Base_Start_IT(BLINK_TIMER) != HAL_OK) Error_Handler();
	}

	auto blink_callback() -> void {
		light.toggle();
		display("Pin State: " + light.read_str());
	}

}


extern "C" {

	void Init() {
		card::init();
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
		if (htim == BLINK_TIMER) {
			card::blink_callback();
		}
	}

}
