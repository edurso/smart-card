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
	bool initialized{};
	bool write{};
	std::size_t count{};

	auto init_callback() -> void {
		smart_card = SmartCard(
			SPI_H,
			I2C_H,
			CARD_READ_TIMER,
			GPIOPin(GPIOA, GPIO_PIN_0),
			GPIOPin(GPIOA, GPIO_PIN_4)
		);
		smart_card.init();
		initialized = true;
	}

	auto card_read_callback() -> void {
		if (!initialized) return;

		// if (!write && (count > 10)) {
		// 	write = true;
		//
		// 	const auto data = "You just got rick rolled hahahahahahahahahahaha";
		// 	// const auto data = "Never gonna give you up Never gonna let you down Never gonna run around and desert you Never gonna make you cry Never gonna say goodbye Never gonna tell a lie and hurt you";
		//
		// 	if (const auto result = smart_card.card_write(data)) {
		// 		if (*result == SUCCESS) {
		// 			debug("Write successful");
		// 		} else {
		// 			debug("Failed to write to card");
		// 		}
		// 	} else {
		// 		debug("Could not detect card to write");
		// 		write = false;
		// 	}
		//
		// }
		// debug("Periodic Callback");
		if (const auto data = smart_card.card_read()) {
			debug("Card Found: \"" + *data + "\"");
		} else {
			debug("No Card Found");
		}
		++count;
	}

	auto imu_interrupt_callback() -> void {
		// smart_card.fired();
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
