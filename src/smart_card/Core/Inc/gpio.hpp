#pragma once

#include <cstdint>
#include <string>

#include "main.h"

namespace card {

	/**
	 * Object representing GPIO pin
	 */
	class GPIOPin {
		GPIO_TypeDef* port{};
		std::uint16_t pin{};
	public:
		GPIOPin() = default;
		GPIOPin(GPIO_TypeDef* port, const std::uint16_t pin) : port{port}, pin{pin} {}

		/**
		 * Read a GPIO_PinState from the pin
		 * @return The value of the pin (SET or RESET)
		 */
		[[nodiscard]] auto read() const -> GPIO_PinState {
			return HAL_GPIO_ReadPin(port, pin);
		}

		/**
		 * Read a GPIO_PinState from the pin as a string
		 * @return The value of the pin (SET or RESET)
		 */
		[[nodiscard]] auto read_str() const -> std::string {
			if (this->read() == GPIO_PIN_SET) {
				return "SET";
			}
			return "RESET";
		}

		/**
		 * Write a GPIO_PinState to the pin
		 * @param value pin state (SET or RESET) to be written
		 */
		auto write(const GPIO_PinState value) const -> void {
			HAL_GPIO_WritePin(port, pin, value);
		}

		/**
		 * Toggle the gpio pin
		 *
		 * SET -> RESET
		 *
		 * RESET -> SET
		 */
		auto toggle() const -> void {
			HAL_GPIO_TogglePin(port, pin);
		}
	};

}
