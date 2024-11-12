#pragma once

#include <cstdint>
#include <sys/types.h>

#include "debug.hpp"

#include "main.h"

//MF522 Command word
#define PCD_IDLE              0x00               //NO action; Cancel the current command
#define PCD_AUTHENT           0x0E               //Authentication Key
#define PCD_RECEIVE           0x08               //Receive Data
#define PCD_TRANSMIT          0x04               //Transmit data
#define PCD_TRANSCEIVE        0x0C               //Transmit and receive data,
#define PCD_RESETPHASE        0x0F               //Reset
#define PCD_CALCCRC           0x03               //CRC Calculate

// Mifare_One card command word
# define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
# define PICC_REQALL          0x52               // find all the cards antenna area
# define PICC_ANTICOLL        0x93               // anti-collision
# define PICC_SElECTTAG       0x93               // election card
# define PICC_AUTHENT1A       0x60               // authentication key A
# define PICC_AUTHENT1B       0x61               // authentication key B
# define PICC_READ            0x30               // Read Block
# define PICC_WRITE           0xA0               // write block
# define PICC_DECREMENT       0xC0               // debit
# define PICC_INCREMENT       0xC1               // recharge
# define PICC_RESTORE         0xC2               // transfer block data to the buffer
# define PICC_TRANSFER        0xB0               // save the data in the buffer
# define PICC_HALT            0x50               // Sleep


//And MF522 The error code is returned when communication
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2


//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG
#define     Reserved20            0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34            0x3F
//-----------------------------------------------
#define MAX_LEN 16

namespace card {
    
    inline auto check_spi(const HAL_StatusTypeDef status) -> void {
        if (status != HAL_SPI_ERROR_NONE) {
            Error_Handler();
        }
    }
    
    inline auto transact(SPI_HandleTypeDef* hspi, const std::uint8_t addr) -> std::uint8_t{
        uint8_t rx;
        check_spi(HAL_SPI_TransmitReceive(hspi, &addr, &rx, 1, 0xFFFFFFFF));
        return rx;
    }
    
    class RC522 {
        SPI_HandleTypeDef* hspi{};
        GPIOPin select_pin{};
        GPIOPin reset_pin{};

        auto select() const -> void {
            select_pin.write(GPIO_PIN_RESET);
        }

        auto deselect() const -> void {
            select_pin.write(GPIO_PIN_SET);
        }

        auto write(const std::uint8_t addr, const std::uint8_t data) const -> void {
            select();
            transact(hspi, ((addr << 1) & 0x7E));
            transact(hspi, data);
            deselect();
        }

        auto write(const std::uint8_t addr, const std::uint32_t count, const std::uint8_t *data) const -> void {
            select();
            transact(hspi, ((addr << 1) & 0x7E));
            for (std::uint32_t i = 0; i < count; i++) {
                transact(hspi, data[i]);
            }
            deselect();
        }

        [[nodiscard]] auto read(const std::uint8_t addr) const -> std::uint8_t {
            select();
            transact(hspi, (((addr << 1) & 0x7E) | 0x80));
            const auto value = transact(hspi, 0x00);
            deselect();
            return value;
        }

        auto read(std::uint8_t addr, std::uint32_t count, std::uint8_t *data, const std::uint8_t rx_align) const -> void {
            addr = 0x80 | (addr & 0x7E); // TODO shift by 1?
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

        auto mask_set(const std::uint8_t addr, const std::uint8_t mask) const -> void {
            write(addr, read(addr) | mask);
        }

        auto mask_clear(const std::uint8_t addr, const std::uint8_t mask) const -> void {
            write(addr, read(addr) & ~mask);
        }

    public:
        RC522() = default;
        RC522(SPI_HandleTypeDef* hspi, const GPIOPin select_pin, const GPIOPin reset_pin)
        : hspi(hspi), select_pin(select_pin), reset_pin{reset_pin} {
            deselect();
            reset_pin.write(GPIO_PIN_SET);
        }

        auto test_io() const -> void {
            constexpr std::uint8_t addr = 0x2C;
            constexpr std::uint8_t data = 0xDE;

            debug(std::to_string(addr) + ": " + std::to_string(read(addr)));
            write(addr, data);
            debug(std::to_string(addr) + ": " + std::to_string(read(addr)));
        }

        auto antenna_on() const -> void {
            mask_set(TxControlReg, 0x03);
        }

        auto antenna_off() const -> void {
            mask_clear(TxControlReg, 0x03);
        }

        auto reset() const -> void {
            write(CommandReg, PCD_RESETPHASE);
        }

        auto init() const -> void {
            reset();

            write(TModeReg, 0x8D);
            write(TPrescalerReg, 0x3E);
            write(TReloadRegL, 30);
            write(TReloadRegH, 0);
            write(TxAutoReg, 0x40);
            write(ModeReg, 0x3D);

            antenna_on();
        }

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


        auto to_card(const std::uint8_t command, const std::uint8_t *send_data, const std::uint8_t send_len, std::uint8_t *back_data, std::uint32_t *back_len) const -> std::uint8_t {

            std::uint8_t status = MI_ERR;
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
            const std::uint8_t status = to_card(PCD_TRANSCEIVE, buf, 9, buf, &recv_bits);
            if ((status == MI_OK) && (recv_bits == 0x18)) {
                size = buf[0];
            } else {
                size = 0;    
            }

            return size;
        }

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

        auto halt() const -> void {
            std::uint32_t un_len;
            std::uint8_t buf[4];

            buf[0] = PICC_HALT;
            buf[1] = 0;
            calc_crc(buf, 2, &buf[2]);

            to_card(PCD_TRANSCEIVE, buf, 4, buf, &un_len);
        }

    };
    
}
