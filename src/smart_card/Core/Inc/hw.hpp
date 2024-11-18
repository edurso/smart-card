#pragma once

#include "main.h"

namespace card {

	/**
	 * Verify a HAL function has exited successfully
	 * @param status the returned HAL_StatusTypeDef of the called HAL function
	 */
	inline auto check(const HAL_StatusTypeDef status) -> void {
		if (status != HAL_OK) {
			debug("Entering Error Handler");
			Error_Handler();
		}
	}

}
