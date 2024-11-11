#include "card.hpp"

#include "main.h"

// Handles defined in IOC
extern TIM_HandleTypeDef htim6; // 1 MHz
extern TIM_HandleTypeDef htim7; // 1 Hz

extern SPI_HandleTypeDef hspi1;

// Rename handles
#define CARD_READ_TIMER &htim6
#define BLINK_TIMER &htim7
#define SPI_H &hspi1


namespace card {

	GPIOPin light;
	RC522 rfid;

	auto init() -> void {
		display("initializing...");
		light = GPIOPin(GPIOB, GPIO_PIN_3);
		rfid = RC522(SPI_H, GPIOPin(GPIOA, GPIO_PIN_0));

		// Start Timer Interrupts
		check(HAL_TIM_Base_Start_IT(BLINK_TIMER));
		check(HAL_TIM_Base_Start_IT(CARD_READ_TIMER));

		display("initialized");
	}

	auto blink_callback() -> void {
		light.toggle();
		// display("Pin State: " + light.read_str());
	}

	auto card_read_callback() -> void {
		display("polling");
		std::uint8_t id[4];
		std::uint8_t type;

		if (rfid.poll(id, &type) == MI_OK) {
			display("success: card id: " + getHexId(id));
		} else {
			display("error: card id: " + getHexId(id));
		}

	}

}


extern "C" {

	void Init() {
		card::init();
	}

	// ReSharper disable once CppParameterMayBeConstPtrOrRef
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
		if (htim == BLINK_TIMER) {
			card::blink_callback();
		} else if (htim == CARD_READ_TIMER) {
			// card::card_read_callback();
		}
	}

}
