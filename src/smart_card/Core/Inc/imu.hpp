#pragma once

#include <cstdint>

#include "main.h"


namespace card {

    /**
     * Object Representing the LSM303DLHC
     */
    class IMU {
        static constexpr std::uint8_t ADDR =            0x19;
        static constexpr std::uint8_t ADDR_W =          ADDR << 1;
        static constexpr std::uint8_t ADDR_R =          ADDR_W | 0x01;

        static constexpr std::uint8_t CTRL_REG1_A =     0x20;
        static constexpr std::uint8_t CTRL_REG2_A =     0x21;
        static constexpr std::uint8_t CTRL_REG3_A =     0x22;
        static constexpr std::uint8_t CTRL_REG4_A =     0x23;
        static constexpr std::uint8_t CTRL_REG5_A =     0x24;
        static constexpr std::uint8_t CTRL_REG6_A =     0x25;
        static constexpr std::uint8_t REFERENCE_A =     0x26;
        static constexpr std::uint8_t STATUS_REG_A =    0x27;
        static constexpr std::uint8_t OUT_X_L_A =       0x28;
        static constexpr std::uint8_t OUT_X_H_A =       0x29;
        static constexpr std::uint8_t OUT_Y_L_A =       0x2A;
        static constexpr std::uint8_t OUT_Y_H_A =       0x2B;
        static constexpr std::uint8_t OUT_Z_L_A =       0x2C;
        static constexpr std::uint8_t OUT_Z_H_A =       0x2D;
        static constexpr std::uint8_t FIFO_CTRL_REG_A = 0x2E;
        static constexpr std::uint8_t FIFO_SRC_REG_A =  0x2F;
        static constexpr std::uint8_t INT1_CFG_A =      0x30;
        static constexpr std::uint8_t INT1_SOURCE_A =   0x31;
        static constexpr std::uint8_t INT1_THS_A =      0x32;
        static constexpr std::uint8_t INT1_DURATION_A = 0x33;
        static constexpr std::uint8_t INT2_CFG_A =      0x34;
        static constexpr std::uint8_t INT2_SOURCE_A =   0x35;
        static constexpr std::uint8_t INT2_THS_A =      0x36;
        static constexpr std::uint8_t INT2_DURATION_A = 0x37;
        static constexpr std::uint8_t CLICK_CFG_A =     0x38;
        static constexpr std::uint8_t CLICK_SRC_A =     0x39;
        static constexpr std::uint8_t CLICK_THS_A =     0x3A;
        static constexpr std::uint8_t TIME_LIMIT_A =    0x3B;
        static constexpr std::uint8_t TIME_LATENCY_A =  0x3C;
        static constexpr std::uint8_t TIME_WINDOW_A =   0x3D;
        static constexpr std::uint8_t CRA_REG_M =       0x00;
        static constexpr std::uint8_t CRB_REG_M =       0x01;
        static constexpr std::uint8_t MR_REG_M =        0x02;
        static constexpr std::uint8_t OUT_X_H_M =       0x03;
        static constexpr std::uint8_t OUT_X_L_M =       0x04;
        static constexpr std::uint8_t OUT_Z_H_M =       0x05;
        static constexpr std::uint8_t OUT_Z_L_M =       0x06;
        static constexpr std::uint8_t OUT_Y_H_M =       0x07;
        static constexpr std::uint8_t OUT_Y_L_M =       0x08;
        static constexpr std::uint8_t SR_REG_Mg =       0x09;
        static constexpr std::uint8_t IRA_REG_M =       0x0A;
        static constexpr std::uint8_t IRB_REG_M =       0x0B;
        static constexpr std::uint8_t IRC_REG_M =       0x0C;
        static constexpr std::uint8_t TEMP_OUT_H_M =    0x31;
        static constexpr std::uint8_t TEMP_OUT_L_M =    0x32;

        I2C_HandleTypeDef *hi2c{};
        std::size_t fired_count{};

        /**
         * Write an I2C transaction to the IMU
         * @param buf buffer to be written
         * @param size sie of buffer
         */
        auto write(std::uint8_t *buf, const std::uint16_t size) const -> void {
            check(HAL_I2C_Master_Transmit(hi2c, ADDR_W, buf, size, HAL_MAX_DELAY));
        }

        /**
         * Write a value to a register on the IMU
         * @param reg register address to be written
         * @param val value to be written to the register
         */
        auto write_register(const std::uint8_t reg, const std::uint8_t val) const -> void {
            std::uint8_t buf[12];
            buf[0] = reg;
            buf[1] = val;
            write(buf, 2);
        }

        /**
         * Read an I2C transaction from the IMU
         * @param buf buffer to write then read from the IMU
         * @param size size of buffer
         */
        auto read(std::uint8_t *buf, const std::uint16_t size) const -> void {
            check(HAL_I2C_Master_Transmit(hi2c, ADDR_W, buf, 1, HAL_MAX_DELAY));
            check(HAL_I2C_Master_Receive(hi2c, ADDR_R, buf, size, HAL_MAX_DELAY));
        }

    public:
        IMU() = default;
        explicit IMU(I2C_HandleTypeDef *hi2c)
        : hi2c{hi2c} {
        }

        /**
         * Initialize the IMU for interrupt on sudden motion
         */
        auto init() const -> void {
            write_register(CTRL_REG1_A, 0x97);
            write_register(CTRL_REG2_A, 0x00);
            write_register(CTRL_REG3_A, 0x60);
            write_register(CTRL_REG6_A, 0x41);
            write_register(INT1_CFG_A, 0xD5);
            write_register(INT1_THS_A, 0x0F);
            write_register(INT1_DURATION_A, 0x05);
        }

        /**
         * Read the value of INT1_SRC_A from the IMU and display it to serial
         */
        auto read() const -> void {
            std::uint8_t buf[12];
            buf[0] = INT1_SOURCE_A;
            read(buf, 1);
            debugf("INT1_SRC_A : 0x%x\n\r", buf[0]);
        }

        /**
         * Indicate to the IMU that its interrupt has fired
         */
        auto fired() -> void {
            debug("Fired " + std::to_string(fired_count++));
        }

        /**
         * Get the number of times the IMU instance has fired
         * @return The number of IMU interrupt fires
         */
        [[nodiscard]] auto get_fired() const -> std::size_t {
            return fired_count;
        }
    };

}
