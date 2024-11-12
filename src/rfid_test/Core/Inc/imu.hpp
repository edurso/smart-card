#pragma once

#include <cstdint>
#include <tuple>

#include "main.h"


namespace card {

    /**
     * Object Representing the LSM303DLHC
     */
    class IMU {
        static constexpr std::uint8_t ADDR = 0x19;
        static constexpr std::uint8_t ADDR_W = ADDR << 1;
        static constexpr std::uint8_t ADDR_R = ADDR_W | 0x01;

        static constexpr std::uint8_t CTRL1 = 0x20;

        I2C_HandleTypeDef *hi2c{};

        auto write(std::uint8_t *buf, const std::uint16_t size) const -> void {
            check(HAL_I2C_Master_Transmit(hi2c, ADDR_W, buf, size, HAL_MAX_DELAY));
        }

        auto read(std::uint8_t *buf, const std::uint16_t size) const -> void {
            check(HAL_I2C_Master_Transmit(hi2c, ADDR_W, buf, 1, HAL_MAX_DELAY));
            check(HAL_I2C_Master_Receive(hi2c, ADDR_R, buf, size, HAL_MAX_DELAY));
        }

    public:
        IMU() = default;
        explicit IMU(I2C_HandleTypeDef *hi2c)
        : hi2c{hi2c} {
        }

        auto init() const -> void {
            std::uint8_t buf[12];
            buf[0] = CTRL1;
            buf[1] = 0x97;
            write(&buf[0], 2);
        }

        [[nodiscard]] auto read() const -> std::tuple<double, double, double> {
            std::uint8_t buf[12];
            buf[0] = 0x28 | 0x80;
            read(&buf[0], 6);

            const int16_t x_raw = -((buf[1] << 8) | buf[0]);
            const int16_t y_raw = -((buf[3] << 8) | buf[2]);
            const int16_t z_raw = -((buf[5] << 8) | buf[4]);

            double x = static_cast<double>(x_raw) / 16384.0;
            double y = static_cast<double>(y_raw) / 16384.0;
            double z = static_cast<double>(z_raw) / 16384.0;

            return std::make_tuple(x, y, z);
        }
    };

}
