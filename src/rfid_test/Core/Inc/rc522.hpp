#pragma once

#include <cstdint>

#include "debug.hpp"

#include "main.h"

/* MFRC522 Commands */
#define PCD_IDLE						0x00   // NO action; Cancel the current command
#define PCD_AUTHENT						0x0E   // Authentication Key
#define PCD_RECEIVE						0x08   // Receive Data
#define PCD_TRANSMIT					0x04   // Transmit data
#define PCD_TRANSCEIVE					0x0C   // Transmit and receive data,
#define PCD_RESETPHASE					0x0F   // Reset
#define PCD_CALCCRC						0x03   // CRC Calculate

/* Mifare_One card command word */
#define PICC_REQIDL						0x26   // find the antenna area does not enter hibernation // reqa
#define PICC_REQALL						0x52   // find all the cards antenna area
#define PICC_ANTICOLL					0x93   // anti-collision
#define PICC_SElECTTAG					0x93   // election card
#define PICC_AUTHENT1A					0x60   // authentication key A
#define PICC_AUTHENT1B					0x61   // authentication key B
#define PICC_READ						0x30   // Read Block
#define PICC_WRITE						0xA0   // write block
#define PICC_DECREMENT					0xC0   // debit
#define PICC_INCREMENT					0xC1   // recharge
#define PICC_RESTORE					0xC2   // transfer block data to the buffer
#define PICC_TRANSFER					0xB0   // save the data in the buffer
#define PICC_HALT						0x50   // Sleep

/* MFRC522 Registers */
//Page 0: Command and Status
#define REG_RESERVED00			0x00
#define REG_COMMAND				0x01
#define REG_COMM_IE_N			0x02   // enable and disable interrupt request control bits
#define REG_DIV1_EN				0x03   // enable and disable interrupt request control bits
#define REG_COMM_IRQ			0x04   // interrupt request bits
#define REG_DIV_IRQ				0x05   // interrupt request bits
#define REG_ERROR				0x06   // error bits showing the error status of the last command executed
#define REG_STATUS1				0x07   // communication status bits
#define REG_STATUS2				0x08   // receiver and transmitter status bits
#define REG_FIFO_DATA			0x09   // input and output of 64 byte FIFO buffer
#define REG_FIFO_LEVEL			0x0A   // number of bytes stored in the FIFO buffer
#define REG_WATER_LEVEL			0x0B   // level for FIFO underflow and overflow warning
#define REG_CONTROL				0x0C   // miscellaneous control registers
#define REG_BIT_FRAMING			0x0D   // adjustments for bit-oriented frames
#define REG_COLL				0x0E   // bit position of the first bit-collision detected on the RF interface
#define REG_RESERVED01			0x0F
// Page 1: Command
#define REG_RESERVED10			0x10
#define REG_MODE				0x11   // defines general modes for transmitting and receiving
#define REG_TX_MODE				0x12   // defines transmission data rate and framing
#define REG_RX_MODE				0x13   // defines reception data rate and framing
#define REG_TX_CONTROL			0x14   // controls the logical behavior of the antenna driver pins TX1 and TX2
#define REG_TX_AUTO				0x15   // controls the setting of the transmission modulation
#define REG_TX_SELL				0x16   // selects the internal sources for the antenna driver
#define REG_RX_SELL				0x17   // selects internal receiver settings
#define REG_RX_THRESHOLD		0x18   // selects thresholds for the bit decoder
#define REG_DEMOD				0x19   // defines demodulator settings
#define REG_RESERVED11			0x1A
#define REG_RESERVED12			0x1B
#define REG_MIFARE				0x1C   // controls some MIFARE communication transmit parameters
#define REG_RX					0x1D   // controls some MIFARE communication receive parameters
#define REG_RESERVED14			0x1E
#define REG_SERIALSPEED			0x1F   // selects the speed of the serial UART interface
// Page 2: CFG
#define REG_RESERVED20			0x20
#define REG_CRC_RESULT_M		0x21
#define REG_CRC_RESULT_L		0x22
#define REG_RESERVED21			0x23
#define REG_MOD_WIDTH			0x24
#define REG_RESERVED22			0x25
#define REG_RF_CFG				0x26    // configures the receiver gain
#define REG_GS_N				0x27
#define REG_CWGS_PREG			0x28
#define REG_MODGS_PREG			0x29
#define REG_T_MODE				0x2A    // defines settings for the internal timer
#define REG_T_PRESCALER			0x2B    // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
#define REG_T_RELOAD_H			0x2C    // defines the 16-bit timer reload value
#define REG_T_RELOAD_L			0x2D
#define REG_T_COUNTER_VALUE_H	0x2E    // shows the 16-bit timer value
#define REG_T_COUNTER_VALUE_L	0x2F
// Page 3:TestRegister
#define REG_RESERVED30			0x30
#define REG_TEST_SEL1			0x31    // general test signal configuration
#define REG_TEST_SEL2			0x32    // general test signal configuration
#define REG_TEST_PIN_EN			0x33    // enables pin output driver on pins D1 to D7
#define REG_TEST_PIN_VALUE		0x34    // defines the values for D1 to D7 when it is used as an I/O bus
#define REG_TEST_BUS			0x35    // shows the status of the internal test bus
#define REG_AUTO_TEST			0x36    // controls the digital self-test
#define REG_VERSION				0x37    // shows the software version
#define REG_ANALOG_TEST			0x38    // controls the pins AUX1 and AUX2
#define REG_TEST_ADC1			0x39    // defines the test value for TestDAC1
#define REG_TEST_ADC2			0x3A    // defines the test value for TestDAC2
#define REG_TEST_ADC0			0x3B    // shows the value of ADC I and Q channels
#define REG_RESERVED31			0x3C    // reserved for production tests
#define REG_RESERVED32			0x3D    // reserved for production tests
#define REG_RESERVED33			0x3E    // reserved for production tests
#define REG_RESERVED34			0x3F    // reserved for production tests

#define MFRC522_MAX_LEN			16

namespace card {

    typedef enum {
        MI_OK = 0,
        MI_NOTAGERR,
        MI_ERR,
        MI_TIMEOUT,
    } status_t;

    class RC522 {
        SPI_HandleTypeDef* hspi{};
        GPIOPin select_pin{};

        auto select() const -> void {
            this->select_pin.write(GPIO_PIN_RESET);
        }

        auto deselect() const -> void {
            this->select_pin.write(GPIO_PIN_SET);
        }

        static auto check_spi(const HAL_StatusTypeDef status) -> void {
            if (status != HAL_SPI_ERROR_NONE) {
                Error_Handler();
            }
        }

        auto write(const std::uint8_t addr, const std::uint8_t data) const -> void {
            this->select();

            const std::uint8_t waddr = (addr << 1) & 0x7E;
            check_spi(HAL_SPI_Transmit(this->hspi, &waddr, 1, HAL_MAX_DELAY));
            check_spi(HAL_SPI_Transmit(this->hspi, &data, 1, HAL_MAX_DELAY));

            this->deselect();
        }

        [[nodiscard]] auto read(const std::uint8_t addr) const -> std::uint8_t {
            this->select();

            std::uint8_t value = 0x00;

            const std::uint8_t raddr = (addr << 1) | 0x80;
            check_spi(HAL_SPI_Transmit(this->hspi, &raddr, 1, HAL_MAX_DELAY));
            check_spi(HAL_SPI_Receive(this->hspi, &value, 1, HAL_MAX_DELAY));

            this->deselect();

            return value;
        }

        auto mask_set(const std::uint8_t addr, const std::uint8_t mask) const -> void {
            this->write(addr, this->read(addr) | mask);
        }

        auto mask_clear(const std::uint8_t addr, const std::uint8_t mask) const -> void {
            this->write(addr, this->read(addr) & ~mask);
        }

    public:
        RC522() = default;
        RC522(SPI_HandleTypeDef* hspi, const GPIOPin select_pin) : hspi(hspi), select_pin(select_pin) {
            this->deselect();

            // Reset board
            this->write(REG_COMMAND, PCD_RESETPHASE);
            HAL_Delay(100);

            // Write configuration
            this->write(REG_T_MODE, 0x8D);
            this->write(REG_T_PRESCALER, 0x3E);
            this->write(REG_T_RELOAD_H, 0x03);
            this->write(REG_T_RELOAD_L, 0xE8);
            this->write(REG_RF_CFG, 0x70); // 48dB gain
            this->write(REG_TX_AUTO, 0x40);
            this->write(REG_MODE, 0x3D);

            // Turn antenna on
            if (!(this->read(REG_TX_CONTROL) & 0x03)) {
                this->mask_set(REG_TX_CONTROL, 0x03);
            }
        }

        auto to_card(
	        const std::uint8_t command,		// the command to execute - one of the PCD_Command enums
	        const std::uint8_t* send_data,  // pointer to the data to transfer to the FIFO
	        const std::uint8_t send_len,    // number of bytes to transfer to the FIFO
        	std::uint8_t* back_data,		// NULL or pointer to buffer if data should be read back after executing the command
        	std::uint16_t* back_len			// in: max number of bytes to write to *backData, out: the number of bytes returned
        ) const -> status_t {
			status_t status = MI_ERR;
			std::uint8_t irq_en = 0x00;
			std::uint8_t wait_irq = 0x00;
			std::uint8_t n;
			std::uint16_t i;

			switch (command) {
				case PCD_AUTHENT: {
					irq_en = 0x12;
					wait_irq = 0x10; // bit 4
					break;
				}
				case PCD_TRANSCEIVE: {
					irq_en = 0x77; //
					wait_irq = 0x30; // bit 4 IdleIRq, 5 RxIRq
					break;
				}
				default:
					break;
			}

			this->write(REG_COMM_IE_N, irq_en | 0x80);
			this->write(REG_COMMAND, PCD_IDLE); // Stop any active command.
			this->mask_clear(REG_COLL, 0x80); // clear collision register
			//this->mask_clear(REG_COMM_IRQ, 0x80); // Clear all seven interrupt request bits
			this->write(REG_COMM_IRQ, 0x7F); // Clear all seven interrupt request bits via ComIrqReg[7] - Set1, when 0, clear interrupts
			this->mask_set(REG_FIFO_LEVEL, 0x80); // FlushBuffer = 1, FIFO initialization
			//this->write(REG_BIT_FRAMING, 0x00); // make sure to clear bit adjustments (should be calculated though, missing some parameters)

			// Writing data to the FIFO
			for (i = 0; i < send_len; i++) {
				this->write(REG_FIFO_DATA, send_data[i]);
			}

			// Execute the command
			this->write(REG_COMMAND, command);
			if (command == PCD_TRANSCEIVE) {
				this->mask_set(REG_BIT_FRAMING, 0x80);		// StartSend=1, transmission of data starts
			}

			// Waiting to receive data to complete
			i = 36000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
			do {
				//CommIrqReg[7..0]
				//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
				n = this->read(REG_COMM_IRQ);
				i--;
			} while ((i != 0) // i=0 is timeout
						&& !(n & 0x01) // timer interrupt - nothing received in 25ms
						&& !(n & wait_irq) // one of the interrupts that signal success has been sent
					);
			this->mask_clear(REG_BIT_FRAMING, 0x80); // StartSend=0

			std::uint8_t errorRegValue = 0x00;
			errorRegValue = this->read(REG_ERROR);
			if (errorRegValue & 0x13) {	// BufferOvfl ParityErr ProtocolErr
				status = MI_ERR;
				return status;
			}

			if (i == 0) {
				return MI_TIMEOUT;
			}

			if (n & 0x01 && !(n&wait_irq)) {
				return MI_TIMEOUT;
			}

			if (i != 0)  {
				if (!(this->read(REG_ERROR) & 0x1B)) {
					status = MI_OK;
					if (command == PCD_TRANSCEIVE) {
						n = this->read(REG_FIFO_LEVEL);
						const std::uint8_t last_bits = this->read(REG_CONTROL) & 0x07;

						if (n == 0) {
							n = 1;
						}

						if (last_bits) {
							*back_len = (n - 1) * 8 + last_bits;
						} else {
							*back_len = n * 8;
						}

						if (n > MFRC522_MAX_LEN) {
							n = MFRC522_MAX_LEN;
						}

						// Reading the received data in FIFO
						for (i = 0; i < n; i++) {
							back_data[i] = this->read(REG_FIFO_DATA);
						}
					}
				} else {
					return MI_ERR;
				}
			}

			if (errorRegValue & 0x08) { // CollErr
				return MI_ERR;
			}

			return status;
		}

        auto request(const std::uint8_t req_mode, std::uint8_t* tag_type) const -> status_t {
            uint16_t back_bits; // The received data bits

            this->write(REG_BIT_FRAMING, 0x07); // TxLastBists = BitFramingReg[2..0]	???

            tag_type[0] = req_mode;
            status_t status = this->to_card(PCD_TRANSCEIVE, tag_type, 1, tag_type, &back_bits);

            if (status == MI_OK && back_bits != 0x10) {
                status = MI_ERR;
            }
            return status;
        }

    	auto anti_collision(std::uint8_t* serNum) const -> status_t {
		    std::uint16_t unLen;

        	this->write(REG_BIT_FRAMING, 0x00); // TxLastBists = BitFramingReg[2..0]

        	serNum[0] = PICC_ANTICOLL;
        	serNum[1] = 0x20;
        	status_t status = this->to_card(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

        	if (status == MI_OK) {
		        std::uint8_t ser_num_check = 0;
		        std::uint8_t i;
		        //Check card serial number
        		for (i = 0; i < 4; i++) {
        			ser_num_check ^= serNum[i];
        		}
        		if (ser_num_check != serNum[i]) {
        			status = MI_ERR;
        		}
        	}
        	return status;
        }

    	auto calculate_crc(const std::uint8_t* p_in_data, const std::uint8_t len, std::uint8_t* p_out_data) const -> status_t {
        	std::uint8_t i, n;

        	this->mask_clear(REG_DIV_IRQ, 0x04); // CRCIrq = 0
        	this->mask_set(REG_FIFO_LEVEL, 0x80); // Clear the FIFO pointer
        	this->write(REG_COMMAND, PCD_IDLE); // Stop any active command.

        	// Writing data to the FIFO
        	for (i = 0; i < len; i++) {
        		this->write(REG_FIFO_DATA, *(p_in_data+i));
        	}
        	this->write(REG_COMMAND, PCD_CALCCRC);

        	// Wait CRC calculation is complete
        	i = 0xFF;
        	do {
        		n = this->read(REG_DIV_IRQ);
        		i--;
        	} while ((i!=0) && !(n&0x04)); // CRCIrq = 1

        	if (i == 0) {
        		return MI_TIMEOUT;
        	}

        	// Read CRC calculation result
        	p_out_data[0] = this->read(REG_CRC_RESULT_L);
        	p_out_data[1] = this->read(REG_CRC_RESULT_M);

        	return MI_OK;
        }
    	
    	auto select_tag(const std::uint8_t* ser_num, std::uint8_t* type) const -> status_t {
		    std::uint8_t size;
        	std::uint16_t recv_bits;
        	std::uint8_t buffer[9];
        	std::uint8_t sak[3] = {0};

        	buffer[0] = PICC_SElECTTAG;
        	buffer[1] = 0x70;
        	for (std::uint8_t i = 0; i < 4; i++) {
        		buffer[i+2] = *(ser_num+i);
        	}
        	buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]; // Calculate BCC - Block Check Character
        	status_t status = this->calculate_crc(buffer, 7, &buffer[7]);

        	if (status != MI_OK) {
        		return status;
        	}

        	status = this->to_card(PCD_TRANSCEIVE, buffer, 9, sak, &recv_bits);

        	if ((status == MI_OK) && (recv_bits == 0x18)) {
        		size = buffer[0];
        	} else {
        		size = 0;
        	}

        	if (recv_bits != 24) { // SAK must be exactly 24 bits (1 byte + CRC_A).
        		return MI_ERR;
        	}

        	*type = sak[0];

        	return status;
        }

    	auto halt() const -> void {
        	std::uint16_t un_len;
        	std::uint8_t buff[4];

        	buff[0] = PICC_HALT;
        	buff[1] = 0;
        	this->calculate_crc(buff, 2, &buff[2]);

        	this->to_card(PCD_TRANSCEIVE, buff, 4, buff, &un_len);
        }

        [[nodiscard]] auto poll(std::uint8_t* id, std::uint8_t* type) const -> status_t {
            status_t status = this->request(PICC_REQIDL, id);

            if (status == MI_OK) {
                //Card detected
                //Anti-collision, return card serial number 4 bytes
                status = this->anti_collision(id);
                //select, return sak and crc
                status = this->select_tag(id, type);
            }

            this->halt(); // Command card into hibernation

            return status;

        }


    };

}
