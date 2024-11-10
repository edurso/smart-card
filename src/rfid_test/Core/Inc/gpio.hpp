#pragma once

#include <cstdint>
#include <string>

#include "main.h"

namespace card {

	class GPIOPin {
		GPIO_TypeDef* port{};
		std::uint16_t pin{};
	public:
		GPIOPin() = default;
		GPIOPin(GPIO_TypeDef* port, std::uint16_t pin) : port{port}, pin{pin} {}

		auto read() const -> GPIO_PinState {
			return HAL_GPIO_ReadPin(port, pin);
		}

		auto read_str() const -> std::string {
			if (this->read() == GPIO_PIN_SET) {
				return "SET";
			}
			return "RESET";
		}

		auto write(GPIO_PinState value) const -> void {
			HAL_GPIO_WritePin(port, pin, value);
		}

		auto toggle() const -> void {
			HAL_GPIO_TogglePin(port, pin);
		}
	};

}
