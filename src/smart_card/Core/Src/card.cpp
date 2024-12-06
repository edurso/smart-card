#include "card.hpp"

#include "main.h"

// Handles defined in IOC
extern TIM_HandleTypeDef htim1; // Variable Hz
extern TIM_HandleTypeDef htim7; // 100 Hz

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

// Rename handles
#define SPEAKER_TIMER &htim1
#define INT_TIMER &htim7
#define SPI_H &hspi1
#define I2C_H &hi2c1


namespace card {

	SmartCard smart_card;
	bool initialized{};
	bool write{};

    // Format: "John Doe|john@doe.com|+1 (123) 567-1234|These are some notes|~"
    // const auto data = "John Doe|john@doe.com|+1 (123) 567-1234|These are some notes|~";
    const std::string data = "Eric D'Urso|edurso@umich.edu|+1 (734) 560-3417|Some Random EECS373 Student|~";
    // const std::string data = "Luke Nelson|lukenels@umich.edu|+1 (734) 892-6993|Some Random EECS373 Student|~";

	auto init_callback() -> void {
		smart_card = SmartCard(
		    data,
			SPI_H,
			I2C_H,
			INT_TIMER,
			SPEAKER_TIMER,
			TIM_CHANNEL_1,
			GPIOPin(GPIOA, GPIO_PIN_0),
			GPIOPin(GPIOA, GPIO_PIN_5),
			GPIOPin(GPIOA, GPIO_PIN_11),
			GPIOPin(GPIOA, GPIO_PIN_12),
			GPIOPin(GPIOB, GPIO_PIN_7),
			GPIOPin(GPIOB, GPIO_PIN_4)
		);
		smart_card.init();
		initialized = true;
	}

    auto write_card() -> void {
	    if (!write) {
	    	write = true;

	    	// const auto data = "";
	    	// const auto data = "This is some data on the card.";
	    	// const auto data = "This is some different data on a different card.";
	    	// const auto data = "This is some different data on a keychain tag.";
	    	// const auto data = "This data is on the card. The card can Store 1KB of Data!";
	    	// const auto data = "\n\rNever gonna give you up\n\rNever gonna let you down\n\rNever gonna run around and desert you\n\rNever gonna make you cry\n\rNever gonna say goodbye\n\rNever gonna tell a lie and hurt you\n\r";
	    	// const auto data = "According to all known laws of aviation, there is no way a bee should be able to fly. Its wings are too small to get its fat little body off the ground. The bee, of course, flies anyway because bees don't care what humans think is impossible. Yellow, black. Yellow, black. Yellow, black. Yellow, black. Ooh, black and yellow! Let's shake it up a little. Barry! Breakfast is ready! Coming! Hang on a second. Hello? Barry? Adam? Can you believe this is happening? I can't. I'll pick you up. Looking sharp. Use the stairs, Your father paid good money for those. Sorry. I'm excited. Here's the graduate. We're very proud of you, son. A perfect report card, all B's. Very proud. Ma! I got a thing going here. You got lint on your fuzz. Ow! That's me!";

	    	if (const auto result = smart_card.write_card(data)) {
	    		if (*result == SUCCESS) {
	    			debug("Card Found: Write Successful");
	    		} else {
	    			debug("Failed to write to card");
	    		}
	    	} else {
	    		debug("No Card Found");
	    		write = false;
	    	}

	    }
	}

	auto imu_interrupt_callback() -> void {
	    if (!initialized) return;

	    // write_card();
		smart_card.motion_detected();
	}

	auto noise_callback() -> void {
	    if (!initialized) return;

	    // NOTE disable on write_card() call
	    smart_card.update_speaker();
	    smart_card.update_card_read_state();
	}

    auto get_data(const req_t req) -> contact_t {
	    if (!initialized) return Contact().get_contact_t();
	    return smart_card.get_data(req);
	}

}


extern "C" {

	void Init() {
		card::init_callback();
	}

    contact_t get_data(const req_t req) {
	    return card::get_data(req);
	}

	// ReSharper disable once CppParameterMayBeConstPtrOrRef
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
		if (htim == INT_TIMER) {
			card::noise_callback();
		}
	}

	// ReSharper disable once CppParameterMayBeConst
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		if (GPIO_Pin == GPIO_PIN_1) {
			card::imu_interrupt_callback();
		}
	}

}
