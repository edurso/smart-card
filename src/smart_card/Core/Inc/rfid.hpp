#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <algorithm>
#include <tuple>

#include "debug.hpp"

#include "main.h"


namespace card {

    /**
     * Check that an SPI transaction executed successfully
     * @param status The status returned from the HAL SPI transaction
     */
    inline auto check_spi(const HAL_StatusTypeDef status) -> void {
        if (status != HAL_SPI_ERROR_NONE) {
            debug("SPI Error Encountered");
            Error_Handler();
        }
    }

    /**
     * Issue an SPI transaction
     * @param hspi SPI handle to use for transaction
     * @param addr address to access on device over SPI
     * @return the byte received from the SPI transaction
     */
    inline auto transact(SPI_HandleTypeDef* hspi, const std::uint8_t addr) -> std::uint8_t {
        uint8_t rx;
        check_spi(HAL_SPI_TransmitReceive(hspi, &addr, &rx, 1, 0xFFFFFFFF));
        return rx;
    }

    /**
     * Represents a transaction state on the card.
     */
    enum CardTransaction {
        SUCCESS,
        FAILURE,
    };
    
    /**
     * Object Representing the RC522 (MFRC522 Chip)
     */
    class RFID {
        static constexpr std::uint8_t MAX_LEN =         16;

        // MF522 Command word
        static constexpr std::uint8_t PCD_IDLE =        0x00; // NO action; Cancel the current command
        static constexpr std::uint8_t PCD_AUTHENT =     0x0E; // Authentication Key
        static constexpr std::uint8_t PCD_RECEIVE =     0x08; // Receive Data
        static constexpr std::uint8_t PCD_TRANSMIT =    0x04; // Transmit data
        static constexpr std::uint8_t PCD_TRANSCEIVE =  0x0C; // Transmit and receive data,
        static constexpr std::uint8_t PCD_RESETPHASE =  0x0F; // Reset
        static constexpr std::uint8_t PCD_CALCCRC =     0x03; // CRC Calculate

        // Mifare_One card command word
        static constexpr std::uint8_t PICC_REQIDL =     0x26; // find the antenna area does not enter hibernation
        static constexpr std::uint8_t PICC_REQALL =     0x52; // find all the cards antenna area
        static constexpr std::uint8_t PICC_ANTICOLL =   0x93; // anti-collision
        static constexpr std::uint8_t PICC_SElECTTAG =  0x93; // election card
        static constexpr std::uint8_t PICC_AUTHENT1A =  0x60; // authentication key A
        static constexpr std::uint8_t PICC_AUTHENT1B =  0x61; // authentication key B
        static constexpr std::uint8_t PICC_READ =       0x30; // Read Block
        static constexpr std::uint8_t PICC_WRITE =      0xA0; // write block
        static constexpr std::uint8_t PICC_DECREMENT =  0xC0; // debit
        static constexpr std::uint8_t PICC_INCREMENT =  0xC1; // recharge
        static constexpr std::uint8_t PICC_RESTORE =    0xC2; // transfer block data to the buffer
        static constexpr std::uint8_t PICC_TRANSFER =   0xB0; // save the data in the buffer
        static constexpr std::uint8_t PICC_HALT =       0x50; // Sleep


        //And MF522 The error code is returned when communication
        static constexpr std::uint8_t MI_OK =           0;
        static constexpr std::uint8_t MI_NOTAGERR =     1;
        static constexpr std::uint8_t MI_ERR =          2;


        //------------------MFRC522 Register---------------
        // Page 0:Command and Status
        static constexpr std::uint8_t Reserved00 =      0x00;
        static constexpr std::uint8_t CommandReg =      0x01;
        static constexpr std::uint8_t CommIEnReg =      0x02;
        static constexpr std::uint8_t DivlEnReg =       0x03;
        static constexpr std::uint8_t CommIrqReg =      0x04;
        static constexpr std::uint8_t DivIrqReg =       0x05;
        static constexpr std::uint8_t ErrorReg =        0x06;
        static constexpr std::uint8_t Status1Reg =      0x07;
        static constexpr std::uint8_t Status2Reg =      0x08;
        static constexpr std::uint8_t FIFODataReg =     0x09;
        static constexpr std::uint8_t FIFOLevelReg =    0x0A;
        static constexpr std::uint8_t WaterLevelReg =   0x0B;
        static constexpr std::uint8_t ControlReg =      0x0C;
        static constexpr std::uint8_t BitFramingReg =   0x0D;
        static constexpr std::uint8_t CollReg =         0x0E;
        static constexpr std::uint8_t Reserved01 =      0x0F;
        // Page 1:Command
        static constexpr std::uint8_t Reserved10 =      0x10;
        static constexpr std::uint8_t ModeReg =         0x11;
        static constexpr std::uint8_t TxModeReg =       0x12;
        static constexpr std::uint8_t RxModeReg =       0x13;
        static constexpr std::uint8_t TxControlReg =    0x14;
        static constexpr std::uint8_t TxAutoReg =       0x15;
        static constexpr std::uint8_t TxSelReg =        0x16;
        static constexpr std::uint8_t RxSelReg =        0x17;
        static constexpr std::uint8_t RxThresholdReg =  0x18;
        static constexpr std::uint8_t DemodReg =        0x19;
        static constexpr std::uint8_t Reserved11 =      0x1A;
        static constexpr std::uint8_t Reserved12 =      0x1B;
        static constexpr std::uint8_t MifareReg =       0x1C;
        static constexpr std::uint8_t Reserved13 =      0x1D;
        static constexpr std::uint8_t Reserved14 =      0x1E;
        static constexpr std::uint8_t SerialSpeedReg =  0x1F;
        // Page 2:CFG
        static constexpr std::uint8_t Reserved20 =          0x20;
        static constexpr std::uint8_t CRCResultRegM =       0x21;
        static constexpr std::uint8_t CRCResultRegL =       0x22;
        static constexpr std::uint8_t Reserved21 =          0x23;
        static constexpr std::uint8_t ModWidthReg =         0x24;
        static constexpr std::uint8_t Reserved22 =          0x25;
        static constexpr std::uint8_t RFCfgReg =            0x26;
        static constexpr std::uint8_t GsNReg =              0x27;
        static constexpr std::uint8_t CWGsPReg =            0x28;
        static constexpr std::uint8_t ModGsPReg =           0x29;
        static constexpr std::uint8_t TModeReg =            0x2A;
        static constexpr std::uint8_t TPrescalerReg =       0x2B;
        static constexpr std::uint8_t TReloadRegH =         0x2C;
        static constexpr std::uint8_t TReloadRegL =         0x2D;
        static constexpr std::uint8_t TCounterValueRegH =   0x2E;
        static constexpr std::uint8_t TCounterValueRegL =   0x2F;
        // Page 3:TestRegister
        static constexpr std::uint8_t Reserved30 =          0x30;
        static constexpr std::uint8_t TestSel1Reg =         0x31;
        static constexpr std::uint8_t TestSel2Reg =         0x32;
        static constexpr std::uint8_t TestPinEnReg =        0x33;
        static constexpr std::uint8_t TestPinValueReg =     0x34;
        static constexpr std::uint8_t TestBusReg =          0x35;
        static constexpr std::uint8_t AutoTestReg =         0x36;
        static constexpr std::uint8_t VersionReg =          0x37;
        static constexpr std::uint8_t AnalogTestReg =       0x38;
        static constexpr std::uint8_t TestDAC1Reg =         0x39;
        static constexpr std::uint8_t TestDAC2Reg =         0x3A;
        static constexpr std::uint8_t TestADCReg =          0x3B;
        static constexpr std::uint8_t Reserved31 =          0x3C;
        static constexpr std::uint8_t Reserved32 =          0x3D;
        static constexpr std::uint8_t Reserved33 =          0x3E;
        static constexpr std::uint8_t Reserved34 =          0x3F;
        //-----------------------------------------------
        static constexpr std::uint8_t key_a[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        static constexpr std::array<std::uint8_t, 47> blocks = {1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18, 20, 21, 22,
                                                                24, 25, 26, 28, 29, 30, 32, 33, 34, 36, 37, 38, 40, 41, 42,
                                                                44, 45, 46, 48, 49, 50, 52, 53, 54, 56, 57, 58, 60, 61, 62};

        SPI_HandleTypeDef* hspi{};
        DMA_HandleTypeDef* hspi_rx{};
        DMA_HandleTypeDef* hspi_tx{};
        GPIOPin select_pin{};
        GPIOPin reset_pin{};
        bool initialized{};
        bool use_dma{};

        /**
         * Pull Chip Select line low to indicate communication on bus is to RC522
         */
        auto select() const -> void {
            select_pin.write(GPIO_PIN_RESET);
        }

        /**
         * Pull Chip Select line high to indicate communication on bus is not to RC522
         */
        auto deselect() const -> void {
            select_pin.write(GPIO_PIN_SET);
        }

        /**
         * Write a value to the RC522
         * @param addr address to be written
         * @param data data to be written to addr
         */
        auto write(const std::uint8_t addr, const std::uint8_t data) const -> void {
            select();
            transact(hspi, ((addr << 1) & 0x7E));
            transact(hspi, data);
            deselect();
        }

        /**
         * Write a series of bytes to the RC522
         * @param addr address to write
         * @param count number of bytes to write
         * @param data data to be written
         */
        auto write(const std::uint8_t addr, const std::uint32_t count, const std::uint8_t *data) const -> void {
            select();
            transact(hspi, ((addr << 1) & 0x7E));
            for (std::uint32_t i = 0; i < count; i++) {
                transact(hspi, data[i]);
            }
            deselect();
        }

        /**
         * Read a byte of data from the RC522
         * @param addr address of data to read
         * @return the value at the given address
         */
        [[nodiscard]] auto read(const std::uint8_t addr) const -> std::uint8_t {
            select();
            transact(hspi, (((addr << 1) & 0x7E) | 0x80));
            const auto value = transact(hspi, 0x00);
            deselect();
            return value;
        }

        /**
         * Read a series of data from the RC522
         * @param addr address to read
         * @param count number of bytes to read
         * @param data pointer to return data
         * @param rx_align how to align data
         */
        auto read(std::uint8_t addr, std::uint32_t count, std::uint8_t *data, const std::uint8_t rx_align) const -> void {
            addr = 0x80 | (addr & 0x7E);
            std::uint8_t index = 0;
            if (count == 0) return;

            select();
            count--;
            transact(hspi, addr);

            while (index < count) {
                if (index == 0 && rx_align) {
                    std::uint8_t mask = 0;
                    for ( int i = rx_align ; i <= 7 ; i++) {
                        mask |= (1 << i);
                    }
                    const std::uint8_t value = transact(hspi, addr);
                    data[0] = (data[index] & ~mask) | (value & mask);
                }
                else { // Normal case
                    data[index] = transact(hspi, addr);
                }
                index++;
            }
            data[index] = transact(hspi, 0x00);
            deselect();
        }

        /**
         * Set a mask on a register on the RC522
         * @param addr register address
         * @param mask mask
         */
        auto mask_set(const std::uint8_t addr, const std::uint8_t mask) const -> void {
            write(addr, read(addr) | mask);
        }

        /**
         * Clear a mask from a register on the RC522
         * @param addr register address
         * @param mask mask
         */
        auto mask_clear(const std::uint8_t addr, const std::uint8_t mask) const -> void {
            write(addr, read(addr) & ~mask);
        }

        /**
         *
         * @param command command to send
         * @param send_data data for command
         * @param send_len length of send_data
         * @param back_data data received from card
         * @param back_len length of data received
         * @return
         */
        auto to_card(const std::uint8_t command, const std::uint8_t *send_data, const std::uint8_t send_len, std::uint8_t *back_data, std::uint32_t *back_len) const -> std::uint8_t {

            std::uint8_t status;
            std::uint8_t irq_en = 0x00;
            std::uint8_t wait_irq = 0x00;

            switch (command) {
            case PCD_AUTHENT: {
                irq_en = 0x12;
                wait_irq = 0x10;
                break;
            }
            case PCD_TRANSCEIVE: {
                irq_en = 0x77;
                wait_irq = 0x30;
                break;
            }
            default:
                break;
            }

            write(CommIEnReg, (irq_en | 0x80));
            mask_clear(CommIrqReg, 0x80);
            mask_set(FIFOLevelReg, 0x80);
            write(CommandReg, PCD_IDLE);

            for (std::uint32_t i = 0 ; i < send_len ; i++) write(FIFODataReg, send_data[i]);

            write(CommandReg, command);
            if (command == PCD_TRANSCEIVE) mask_set(BitFramingReg, 0x80);

            // wait for receive data to complete
            std::uint32_t i = 2000;
            std::uint8_t n;
            do {
                n = read(CommIrqReg);
                --i;
            } while ((i != 0) && !(n & 0x01) && !(n & wait_irq));

            mask_clear(BitFramingReg, 0x80);

            if (i != 0) {
                if (!(read(ErrorReg) & 0x1B)) {
                    status = MI_OK;
                    if (n & irq_en & 0x01) status = MI_NOTAGERR;
                    if (command == PCD_TRANSCEIVE) {

                        n = read(FIFOLevelReg);
                        if (const std::uint8_t last_bits = read(ControlReg) & 0x07) {
                            *back_len = (n - 1) * 8 + last_bits;
                        } else {
                            *back_len = n * 8;
                        }

                        if (n == 0) n = 1;
                        if (n > MAX_LEN) n = MAX_LEN;

                        for (i = 0 ; i < n ; i++) {
                            back_data[i] = read(FIFODataReg);
                        }
                    }
                } else {
                    status = MI_ERR;
                    debug("Request resulted in error");
                }
            } else {
                status = MI_ERR;
                debug("Request timed out");
            }

            return status;
        }

        /**
         * calculate card CRC per data sheet
         * @param p_in_data input data
         * @param len length of input data
         * @param p_out_data pointer to output data
         */
        auto calc_crc(const std::uint8_t *p_in_data, const std::uint8_t len, std::uint8_t *p_out_data) const -> void {
            std::uint8_t i, n;

            mask_clear(DivIrqReg, 0x04);
            mask_set(FIFOLevelReg, 0x80);
            // write(CommandReg, PCD_IDLE);

            for (i = 0 ; i < len ; i++)
            {
                write(FIFODataReg, *(p_in_data+i));
            }
            write(CommandReg, PCD_CALCCRC);

            i = 0xFF;
            do  {
                n = read(DivIrqReg);
                i--;
            } while ((i != 0) && !(n & 0x04));

            p_out_data[0] = read(CRCResultRegL);
            p_out_data[1] = read(CRCResultRegM);
        }

        /**
         * Turn the RC522's antenna on
         */
        auto antenna_on() const -> void {
            mask_set(TxControlReg, 0x03);
        }

        /**
         * Turn the RC522's antenna off
         */
        auto antenna_off() const -> void {
            mask_clear(TxControlReg, 0x03);
        }

        /**
         * Send a reset command to the RC522
         */
        auto reset() const -> void {
            write(CommandReg, PCD_RESETPHASE);
        }

        /**
         * Request for cards in proximity
         * @param req_mode request mode
         * @param tag_type tag type
         * @return MI_OK if successgful
         */
        auto request(const std::uint8_t req_mode, std::uint8_t *tag_type) const -> std::uint8_t {
            std::uint32_t back_bits;
            write(BitFramingReg, 0x07);
            tag_type[0] = req_mode;

            std::uint8_t status = to_card(PCD_TRANSCEIVE, tag_type, 1, tag_type, &back_bits);
            if ((status != MI_OK) || (back_bits != 0x10)) {
                status = MI_ERR;
            }

            return status;
        }

        /**
         * Send anticoll command to card to receive UID
         * @param ser_num card serial number (UID)
         * @return MI_OK if successfull
         */
        auto anticoll(std::uint8_t *ser_num) const -> std::uint8_t {
            std::uint32_t un_len;

            write(BitFramingReg, 0x00);

            ser_num[0] = PICC_ANTICOLL;
            ser_num[1] = 0x20;
            std::uint8_t status = to_card(PCD_TRANSCEIVE, ser_num, 2, ser_num, &un_len);

            if (status == MI_OK) {
                std::uint8_t i;
                std::uint8_t ser_num_check = 0;
                for (i = 0 ; i < 4 ; i++) {
                    ser_num_check ^= ser_num[i];
                }
                if (ser_num_check != ser_num[i]) status = MI_ERR;
            }

            return status;
        }

        /**
         * Get the tag from the RFID card
         * @param ser_num card serial number (UID)
         * @return MI_OK if successful
         */
        auto select_tag(const std::uint8_t *ser_num) const -> std::uint8_t {
            std::uint8_t size;
            std::uint32_t recv_bits;
            std::uint8_t buf[9];

            buf[0] = PICC_SElECTTAG;
            buf[1] = 0x70;
            for (std::uint8_t i = 0 ; i < 5 ; i++) {
                buf[i+2] = *(ser_num + i);
            }
            calc_crc(buf, 7, &buf[7]); //??
            if (const std::uint8_t status = to_card(PCD_TRANSCEIVE, buf, 9, buf, &recv_bits); (status == MI_OK) && (recv_bits == 0x18)) {
                size = buf[0];
            } else {
                size = 0;    
            }

            return size;
        }

        /**
         * Authenticate to a sector
         * @param auth_mode authentication mode (0x60 in most cases)
         * @param block_addr address of block to authenticate to
         * @param sector_key the sector id to authenticate to
         * @param ser_num serial number (UID) of card
         * @return MI_OK if transaction successful
         */
        auto auth(const std::uint8_t auth_mode, const std::uint8_t block_addr, const std::uint8_t *sector_key, const std::uint8_t *ser_num) const -> std::uint8_t {
            std::uint32_t recv_bits;
            std::uint8_t i;
            std::uint8_t buf[12];

            buf[0] = auth_mode;
            buf[1] = block_addr;
            for (i = 0 ; i < 6 ; i++) {
                buf[i + 2] = *(sector_key + i);
            }
            for (i = 0 ; i < 4 ; i++) {
                buf[i + 8] = *(ser_num + i);
            }
            std::uint8_t status = to_card(PCD_AUTHENT, buf, 12, buf, &recv_bits);

            if ((status != MI_OK) || (!(read(Status2Reg) & 0x08))) {
                status = MI_ERR;   
            }

            return status;
        }

        /**
         * Read a 16 byte block of data from the card
         * @param block_addr address of the block to read
         * @param recv_data pointer to head of data buffer
         * @return MI_OK if the transaction was successfull
         */
        auto read_data(const std::uint8_t block_addr, std::uint8_t *recv_data) const -> std::uint8_t {
            std::uint32_t un_len;

            recv_data[0] = PICC_READ;
            recv_data[1] = block_addr;
            calc_crc(recv_data,2, &recv_data[2]);
            std::uint8_t status = to_card(PCD_TRANSCEIVE, recv_data, 4, recv_data, &un_len);

            if ((status != MI_OK) || (un_len != 0x90)) {
                status = MI_ERR;
            }

            return status;
        }

        /**
         * Write a 16 byte block to the card memory
         * @param block_addr address of the block to write
         * @param write_data the 16 bytes of data to be written
         * @return MI_OK if the transaction was successful
         */
        auto write_data(const std::uint8_t block_addr, const std::uint8_t *write_data) const -> std::uint8_t {
            std::uint32_t recv_bits;
            std::uint8_t buff[18];

            buff[0] = PICC_WRITE;
            buff[1] = block_addr;
            calc_crc(buff, 2, &buff[2]);
            std::uint8_t status = to_card(PCD_TRANSCEIVE, buff, 4, buff, &recv_bits);

            if ((status != MI_OK) || (recv_bits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
                status = MI_ERR;
            }

            if (status == MI_OK) {
                for (std::uint8_t i = 0 ; i < 16 ; i++) {
                    buff[i] = *(write_data+i);
                }
                calc_crc(buff, 16, &buff[16]);
                status = to_card(PCD_TRANSCEIVE, buff, 18, buff, &recv_bits);

                if ((status != MI_OK) || (recv_bits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
                    status = MI_ERR;   
                }
            }

            return status;
        }

        /**
         * Halt the card to set the card to a ready state for future transactions
         */
        auto halt() const -> void {
            std::uint32_t un_len;
            std::uint8_t buf[4];

            buf[0] = PICC_HALT;
            buf[1] = 0;
            calc_crc(buf, 2, &buf[2]);

            to_card(PCD_TRANSCEIVE, buf, 4, buf, &un_len);
        }

    public:
        RFID() = default;
        RFID(
            SPI_HandleTypeDef* hspi,
            DMA_HandleTypeDef* hspi_rx,
            DMA_HandleTypeDef* hspi_tx,
            const bool use_dma,
            const GPIOPin select_pin,
            const GPIOPin reset_pin
            ) :
        hspi(hspi),
        hspi_rx(hspi_rx),
        hspi_tx(hspi_tx),
        select_pin(select_pin),
        reset_pin{reset_pin},
        use_dma(use_dma)
        {
            deselect();
            reset_pin.write(GPIO_PIN_SET);
        }

        /**
         * Test reading and writing to the RC522 module over SPI
         */
        auto test_io() const -> void {
            constexpr std::uint8_t addr = 0x2C;
            constexpr std::uint8_t data = 0xDE;

            debug(std::to_string(addr) + ": " + std::to_string(read(addr)));
            write(addr, data);
            debug(std::to_string(addr) + ": " + std::to_string(read(addr)));
        }

        /**
         * Initialize the RFID module
         */
        auto init() -> void {
            reset();

            write(TModeReg, 0x8D);
            write(TPrescalerReg, 0x3E);
            write(TReloadRegL, 30);
            write(TReloadRegH, 0);
            write(TxAutoReg, 0x40);
            write(ModeReg, 0x3D);

            antenna_on();
            initialized = true;
        }

        /**
         * Read data from the memory on the MIFARE RFID Card
         * @return The data in the 47 readable blocks of the card as a string.
         */
        [[nodiscard]] auto read_card() -> std::optional<std::tuple<CardTransaction, std::string>> {
            if (!initialized) return std::nullopt;

            CardTransaction transaction = SUCCESS;

            std::string data;
            std::uint8_t str[MAX_LEN];

            // Re-Initialize RFID Module
            init();

            // Find Cards Present
            auto status = request(PICC_REQIDL, str);
            if (status != MI_OK) {
                // debug("No Card Found");
                halt();
                return std::nullopt;
            }
            // debug("Card Found");

            // Get Card UID
            status = anticoll(str);
            if (status == MI_OK) {
                // debugf("\tCard UID: %x:%x:%x:%x\n\r", str[0], str[1], str[2], str[3]);
                // const auto capacity = select_tag(str);
                // ReSharper disable once CppExpressionWithoutSideEffects
                select_tag(str);
                std::vector<std::string> block_values{};

                for (const auto& block : blocks) {
                    std::string block_value = std::to_string(block) + ": ";
                    if (std::uint8_t process = auth(0x60, block, key_a, str); process == MI_OK) {
                        std::uint8_t block_data[16];
                        process = read_data(block, block_data);
                        if (process == MI_OK) {
                            std::string line;
                            for (const unsigned char i : block_data) {
                                if (i != 0x00) {
                                    line += i;
                                }
                            }
                            block_value += line;
                            data += line;
                            // debug("\tCard Block " + std::to_string(block) + ": \"" + line + "\"");
                        } else {
                            block_value += "Error Reading Data Packet";
                            transaction = FAILURE;
                        }
                    } else {
                        block_value += "Could Not Authenticate To Block";
                        transaction = FAILURE;
                    }
                    block_values.push_back(block_value);
                }

            } else {
                debug("Could Not Parse Card UID");
                halt();
                transaction = FAILURE;
            }
            // debugf("\tCard UID: %x:%x:%x:%x\n\r", str[0], str[1], str[2], str[3]);

            // Reset For Next Poll
            halt();
            return std::make_tuple(transaction, data);;
        }

        /**
         * Write data to the memory on the MIFARE RFID Card
         * @param data Data to be written to the card (not to exceed 47 blocks x 16 bytes/block)
         * @return An optional card transaction. std::nullopt indicates write was not attempted.
         */
        [[nodiscard]] auto write_card(const std::string& data) -> std::optional<CardTransaction> {
            if (!initialized) return std::nullopt;

            constexpr std::uint8_t block_size = 16;
            if (data.length() > (blocks.size() * block_size)) return std::nullopt;

            std::uint8_t str[MAX_LEN];

            // Re-Initialize RFID Module
            init();

            // Find Cards Present
            auto status = request(PICC_REQIDL, str);
            if (status != MI_OK) {
                halt();
                return std::nullopt;
            }

            // Get Card UID
            status = anticoll(str);
            if (status == MI_OK) {
                // const auto capacity = select_tag(str);
                // ReSharper disable once CppExpressionWithoutSideEffects
                select_tag(str);
                std::vector<std::string> block_values{};
                std::uint8_t block_data[block_size];
                std::size_t block_index = 0;

                for (const auto& block : blocks) {
                    std::string block_value = "Block " + std::to_string(block) + ": ";
                    if (std::uint8_t process = auth(0x60, block, key_a, str); process == MI_OK) {
                        std::ranges::fill(block_data, 0x00);

                        const std::size_t start_idx = block_index * block_size;
                        const std::size_t current_chunk_size = std::min(static_cast<std::size_t>(block_size), data.length() - start_idx);

                        if (start_idx < data.length()) {
                            std::copy_n(data.begin() + static_cast<std::uint8_t>(start_idx), current_chunk_size, block_data);
                        }

                        // try {
                        //     debug("Writing data \"" + data.substr(start_idx, current_chunk_size) + "\"");
                        // } catch (...) {}
                        process = write_data(block, block_data);
                        if (process == MI_OK) {
                            block_value += "Success Writing Data Packet";
                        } else {
                            block_value += "Error Writing Data Packet";
                        }
                    } else {
                        block_value += "Could Not Authenticate To Block";
                    }
                    block_values.push_back(block_value);
                    block_index++;
                }

            } else {
                halt();
                return FAILURE;
            }

            // Reset For Next Poll
            halt();
            return SUCCESS;
        }

    };
    
}
