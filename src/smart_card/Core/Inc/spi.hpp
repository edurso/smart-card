#pragma once

#include <cstdint>

#include "debug.hpp"

#include "main.h"


namespace card {

    enum SPITransferStatus : std::uint8_t {
        WAIT = 0,
        COMPLETE,
        ERROR
    };

    /**
     * Check that an SPI transaction executed successfully
     * @param status The status returned from the HAL SPI transaction
     */
    inline auto check_spi(const HAL_StatusTypeDef status) -> void {
        if (status != HAL_OK) {
            debug("SPI Error Encountered");
            Error_Handler();
        }
    }

    /**
     * Issue an SPI transaction
     * @param hspi SPI handle to use for transaction
     * @param spi_status SPI status handle from interrupt
     * @param addr address to access on device over SPI
     * @return the byte received from the SPI transaction
     */
    inline auto transact(SPI_HandleTypeDef* hspi, const SPITransferStatus* spi_status, const std::uint8_t addr) -> std::uint8_t {
        uint8_t rx = 0;
        // check_spi(HAL_SPI_TransmitReceive(hspi, &addr, &rx, 1, 0xFFFFFFFF));
        const auto init_rx = rx;
        // check_spi(HAL_SPI_TransmitReceive_DMA(hspi, &addr, &rx, 1));
        const auto res = HAL_SPI_TransmitReceive_DMA(hspi, &addr, &rx, 1);

        // bool is_first = true;
        // while (*spi_status == WAIT) {
        //     if (is_first) {
        //         is_first = false;
        //         debugf("Waiting for SPI Data, rx=%u\n\r", rx);
        //     }
        // }
        debugf("SPI Finished, res=%u, rx=%u\n\r", res, rx);

        // switch (*spi_status) {
        // case COMPLETE: {
        //     debug("Completed SPI Transaction");
        //     break;
        // }
        // default: {
        //     debug("FATAL SPI Error Encountered");
        //     Error_Handler();
        //     break;
        // }
        // }

        return rx;
    }

}
