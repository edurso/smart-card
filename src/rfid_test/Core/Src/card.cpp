#include "card.hpp"

#include "main.h"

// Handles defined in IOC
extern TIM_HandleTypeDef htim6; // 1 Hz
extern TIM_HandleTypeDef htim7; // 1 Hz

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

// Rename handles
#define CARD_READ_TIMER &htim6
#define BLINK_TIMER &htim7
#define SPI_H &hspi1
#define I2C_H &hi2c1


namespace card {

	SmartCard smart_card;

	auto init_callback() -> void {
		smart_card = SmartCard(
			SPI_H,
			I2C_H,
			CARD_READ_TIMER,
			GPIOPin(GPIOA, GPIO_PIN_0),
			GPIOPin(GPIOA, GPIO_PIN_4)
		);
		smart_card.init();
	}

	auto card_read_callback() -> void {
		// debug("Periodic Callback");
		smart_card.card_read();
	}

	auto imu_interrupt_callback() -> void {
		smart_card.fired();
		// smart_card.card_read();
	}

}


extern "C" {

	void Init() {
		card::init_callback();
	}

	// ReSharper disable once CppParameterMayBeConstPtrOrRef
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
		if (htim == CARD_READ_TIMER) {
			card::card_read_callback();
		}
	}

	// ReSharper disable once CppParameterMayBeConst
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		if (GPIO_Pin == GPIO_PIN_1) {
			card::imu_interrupt_callback();
		}
	}


}
