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

		// if (!write) {
		// 	write = true;
		//
		// 	const auto data = "";
		// 	// const auto data = "This is some data on the card.";
		// 	// const auto data = "This is some different data on a different card.";
		// 	// const auto data = "This is some different data on a keychain tag.";
		// 	// const auto data = "This data is on the card. The card can Store 1KB of Data!";
		// 	// const auto data = "\n\rNever gonna give you up\n\rNever gonna let you down\n\rNever gonna run around and desert you\n\rNever gonna make you cry\n\rNever gonna say goodbye\n\rNever gonna tell a lie and hurt you\n\r";
		// 	// const auto data = "According to all known laws of aviation, there is no way a bee should be able to fly. Its wings are too small to get its fat little body off the ground. The bee, of course, flies anyway because bees don't care what humans think is impossible. Yellow, black. Yellow, black. Yellow, black. Yellow, black. Ooh, black and yellow! Let's shake it up a little. Barry! Breakfast is ready! Coming! Hang on a second. Hello? Barry? Adam? Can you believe this is happening? I can't. I'll pick you up. Looking sharp. Use the stairs, Your father paid good money for those. Sorry. I'm excited. Here's the graduate. We're very proud of you, son. A perfect report card, all B's. Very proud. Ma! I got a thing going here. You got lint on your fuzz. Ow! That's me!";
		//
		// 	if (const auto result = smart_card.card_write(data)) {
		// 		if (*result == SUCCESS) {
		// 			debug("Card Found: Write Successful");
		// 		} else {
		// 			debug("Failed to write to card");
		// 		}
		// 	} else {
		// 		debug("No Card Found");
		// 		write = false;
		// 	}
		//
		// }
		
		if (const auto data = smart_card.read_card()) {
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
