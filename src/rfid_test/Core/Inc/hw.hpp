#pragma once

#include "main.h"

namespace card {
	inline void check(const HAL_StatusTypeDef status) {
		if (status != HAL_OK) {
			debug("Entering Error Handler");
			Error_Handler();
		}
	}
}
