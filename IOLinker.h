/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/IOLinker
 */

/**
 * @file IOLinker.h
 * @author Julian von Mendel
 * @brief IOLinker class header
 */

#ifndef __IOLINKER_H__
#define __IOLINKER_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#define max(a,b) (a>b?a:b)

#ifdef WIRINGPI
#include <wiringSerial.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#elif defined(ARDUINO)
#include <Stream.h>
#include <SPI.h>
#include <Wire.h>
#elif !defined(__PC)
#define __PC
#include "wiringSerial.h"
#endif

#define __IOLINKER_DEBUG (1) /*!< Activate debugging output */


class IOLinker {
    public:
        /**
         * @brief Function type for the beginTest() callback interface
         */
        typedef uint8_t (*testfunc_t)(unsigned char *s, uint8_t len);
        
        /**
         * @brief Function type for callbacks in general
         */
        typedef void (*callback_t)(void);
        
    private:
        /**
         * @brief Communication interface identifier
         */
        enum interface_mode {
            IOLINKER_INTERFACE_UNSET = 0,
            IOLINKER_UART,
            IOLINKER_SPI,
            IOLINKER_I2C,
            IOLINKER_INTERFACE_CALLBACK,
        } interface_mode = IOLINKER_INTERFACE_UNSET;
        
        testfunc_t interface_testfunc; /*!< testfunc interface function
                                            pointer */
        unsigned char *interface_buf, *interface_buf_reset,
                      *interface_buf_end; /*!< String buffers the testfunc
                                               interface uses to communicate
                                               with the user methods */

#ifdef ARDUINO
        Stream *interface_stream; /*!< Stream object of serial interface */
#endif

#if defined(WIRINGPI) || defined(__PC)
        int interface_fd; /*!< File descriptor for communication interface */
#endif

    public:
#define IOLINKER_BAUDRATE 115200UL
#define __IOLINKER_BAUDRATE_WIRINGPI (B115200)

#if defined(WIRINGPI) || defined(__PC)
        /**
         * @brief Setup serial interface
         * @param Device file
         */
        void beginSerial(const char *dev = "/dev/ttyUSB0");
#endif

#ifdef WIRINGPI
#define __IOLINKER_SPI_CLK (3200000UL)
        
        /**
         * @brief Setup SPI master
         * @param channel SPI channel
         */
        void beginSPI(uint8_t channel = 0);
        
        /**
         * @brief Setup  I2C master
         */
        void beginI2C(void);

#elif defined(ARDUINO)
        /**
         * @brief Setup serial interface
         * @param Stream object for serial interface
         */
        inline void beginStream(Stream &stream)
        {
            interface_mode = IOLINKER_UART;
            interface_stream = &stream;
        }

#define __IOLINKER_SPI_SETTINGS SPISettings(14000000, MSBFIRST, SPI_MODE0)
#define __IOLINKER_SPI_CS (10)        
        /**
         * @brief Setup SPI master
         */
        inline void beginSPI(void)
        {
            interface_mode = IOLINKER_SPI;
            SPI.begin();
            pinMode(__IOLINKER_SPI_CS, OUTPUT);
#ifdef SPI_CLOCK_DIV2
            SPI.setClockDivider(SPI_CLOCK_DIV2);
#else
            SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
            SPI.setDataMode(SPI_MODE0);
        }
        
        /**
         * @brief Setup  I2C master
         */
        inline void beginI2C(void)
        {
            interface_mode = IOLINKER_I2C;
            Wire.begin();
        }

#endif

        /**
         * @brief Initialize communication by means of a callback function
         *      that handles the communication
         *
         * Messages are written into the string pointer *buf, until at message
         * end the provided callback function is run, that provides its reply
         * in the same buffer *buf. Can be used for unit testing.
         */
        inline void beginTest(testfunc_t testfunc, uint8_t *buf, uint8_t size)
        {
            interface_mode = IOLINKER_INTERFACE_CALLBACK;
            interface_testfunc = testfunc;
            interface_buf = buf;
            interface_buf_reset = buf;
            interface_buf_end = buf + size;
        }


        /** Message preparation **/

        /**
         * @brief Node target addresses
         */
        typedef enum target_address {
            IOLINKER_TARGET_ALL = 0x7f, /*!< Target all chips simultaneously. Only
                                    supported in Pro versions. */
            IOLINKER_TARGET_FIRST = 0x00,
            IOLINKER_TARGET_MAX = 0x7f,
        } target_address;

        /**
         * @brief Choose current slave address
         * @param addr Slave address to target
         */
        inline void targetAddress(uint8_t addr)
        {
#ifdef WIRINGPI
            if (interface_mode == IOLINKER_I2C && target_addr != addr) {
                interface_fd = wiringPiI2CSetup(addr);
            }
#endif

            target_addr = addr;
        }

#if 0 // Not supported right now
        /**
         * @brief Turn CRC error detection on or off for following messages
         */
        inline void crc(bool active)
        {
            if (active) {
                cmdbyte |= IOLINKER_BITMASK_CRC_BIT;
            } else {
                cmdbyte &= ~(IOLINKER_BITMASK_CRC_BIT);
            }
        }
#endif

#if defined(WIRINGPI)
        /**
         * @brief Register callback function for interrupt events
         * @param intpin The pin you connected the iolinker INT pin with
         * @param callback Function to call when an iolinker interrupt
         *      occurs
         */
        void registerInterrupt(uint8_t intpin, callback_t callback);
#elif defined(ARDUINO)
        /**
         * @brief Register callback function for interrupt events
         * @param intpin The pin you connected the iolinker INT pin with
         * 
         * Define your interrupt routine e.g. for pin 9 in a
         * ISR (PCINT0_vect) { ... } block.
         * The ISR name needs to be changed depending on the
         * interrupt pin number:
         * ISR (PCINT0_vect) pin change interrupt for D8 to D13
         * ISR (PCINT1_vect) pin change interrupt for A0 to A5
         * ISR (PCINT2_vect) pin change interrupt for D0 to D7
         */
        void registerInterrupt(uint8_t intpin);
#endif

        /** Status codes **/

        /**
         * @brief Return status codes
         */
        typedef enum status_code {
            IOLINKER_STATUS_UNDEFINED = 0x00,
            IOLINKER_STATUS_SUCCESS = 0x01,
            IOLINKER_ERROR = 0x02,
            IOLINKER_ERROR_ARGCOUNT = 0x03,
            IOLINKER_ERROR_PINNUM = 0x04,
            IOLINKER_ERROR_INTERFACE = 0xfc,
            IOLINKER_ERROR_NOREPLY = 0xfd,
            IOLINKER_ERROR_CRC = 0xff,
        } status_code;
        
        /**
         * @brief Retrieve the return status code of last write operation
         */
        inline status_code statusCode(void)
        {
            uint8_t code = IOLinker::readRegister(0x7f);
            if (code == 0) {
                return IOLinker::IOLINKER_ERROR_NOREPLY;
            }
            return (status_code)code;
        }

        /**
         * @brief Check chip availability
         * @return Returns true if the chip replied, and false otherwise
         */
        inline bool available(void)
        {
            return (version() != 0);
        }


        /** Messages **/

        /**
         * @brief Send out messages in a byte string, but mix in current
         *      settings and verify successful execution of all commands
         * @param msglist Byte string with a message byte followed by the
         *      message parameters followed by the next message byte and so on
         * @param size Length of the byte string
         * @return Returns true on success return code of ALL commands
         *      executed
         */
        bool sendBuf(uint8_t *msglist, uint16_t size);

        /**
         * @brief VER command: Retrieve chip version
         * @return Returns the received chip version byte, or 0 if no valid
         *      reply was received
         *
         * Version numbers are encoded in one byte of the value 0x00 -- 0x7f.
         */
        uint16_t version(void);

        /**
         * @brief Derive pin count from chip version byte
         * @return Return pin count
         */
        inline uint16_t pinCount(uint16_t version)
        {
            switch ((argData(version) & 0x0f)) {
                case 0:
                    return 14;
                case 1:
                    return 49;
                case 2:
                    return 64;
                case 3:
                    return 192;
            }
            return 0;
        }

        /**
         * @brief Pin type codes
         */
        typedef enum pin_types {
            IOLINKER_INPUT = 0x00, /*!< Low impedance/tristate pin type */
            //IOLINKER_PULLDOWN = 0x01, /*!< Pulldown input pin type */
            //IOLINKER_PULLUP = 0x02, /*!< Pullup input pin type */
            IOLINKER_OUTPUT = 0x03, /*!< Output pin type */
        } pin_types;

        /**
         * @brief TYP command: Set pin type
         * @param Pin type code
         * @param pin_start First pin to update
         * @param pin_end Last pin to update, or 0 if only one is to be changed
         */
        void setPinType(pin_types type, uint16_t pin_start,
                uint16_t pin_end = 0);
        
        /**
         * @brief REA command: Read register
         * @param addr Register address
         */
        uint8_t readRegister(uint8_t addr);

        /**
         * @brief REA command: Read input state
         * @param pin Pin address to read
         *
         * For pins that are not set as input pin, the state will always be
         * returned as low.
         */
        bool readInput(uint16_t pin);

        /**
         * @brief REA command: Read input pin range and store value in
         *      buffer
         * @param Buffer to read pin states into. 8 pin states are encoded
         *      in one byte, the first one in the MSB of the first byte.
         *      Please guarantee that the buffer is REPLY_MAXMETA_BYTECOUNT
         *      bytes larger than necessary just to hold the read pin states.
         * @param len Buffer size
         * @param pin_start First pin to read
         * @param pin_end Last pin to read, or 0 if only one is to be read
         *
         * For pins that are not set as input pin, the state will always be
         * returned as low.
         */
        void readInput(uint8_t *s, uint8_t len, uint16_t pin_start,
                uint16_t pin_end = 0);

        /**
         * @brief SET command: Set output pin states
         * @param state Pin state to set for ALL output pins
         * @param pin_start First pin to update
         * @param pin_end Last pin to update, or 0 if only one is to be changed
         *
         * Pins that are not of output type are skipped.
         * Note that PWM output is only active for output pins in high state,
         * i.e. this command can also be used to turn PWM on and off.
         */
        void setOutput(bool state, uint16_t pin_start, uint16_t pin_end = 0);

        /**
         * @brief SET command: Sets pin range to high (= 1) or low (= 0) state
         * @param s Buffer to read pin states from; 8 pin states are encoded
         *      per byte, with the first pin state on the MSB of the first
         *      byte. If the byte count is insufficient, the remaining pin
         *      states will be assumed to be low.
         * @param len Buffer size
         * @param pin_start First pin to update
         * @param pin_end Last pin to update, or 0 if only one is to be changed
         *
         * Pins that are not of output type are skipped.
         * Note that PWM output is only active for output pins in high state,
         * i.e. this command can also be used to turn PWM on and off.
         */
        void setOutput(uint8_t *s, uint8_t len, uint16_t pin_start,
                uint16_t pin_end = 0);

        /**
         * @brief SYN command: Begin buffering the write commands that
         *      follow.
         *
         * Use this command before you start preparing a new buffered IO
         * state that you plan to write out at a later point.
         */
        void beginBuffering(void);

        /**
         * @brief TRG command: Execute all buffered IO states (i.e. copy
         *      buffer to current and write out pin states)
         *
         * Use this command to write out the buffered IO states, i.e. change
         * all IO states in the same instant.
         */
        void executeBuffer(void);

        /**
         * @brief LNK command: Link output pin range to input/virtual pin z
         * @param target_pin Target pin address
         * @param pin_start First pin to update
         * @param pin_end Last pin to update, or 0 if only one is to be changed
         *
         * If the last pin number equals 0, only one pin state will be
         * changed. Pins that are not of output type are skipped. If the
         * target pin address is a physical pin, and said physical pin is
         * of output type 0x03, it will be set to tristate input type.
         */
        void link(uint16_t target_pin, uint16_t pin_start, uint16_t pin_end = 0);

        /**
         * @brief PWM command: Set PWM ratio for pin range
         * @param pwm_r PWM ratio between 0 and 127, wherein 127 equals 100% on
         * @param pin_start First pin to update
         * @param pin_end Last pin to update, or 0 if only one is to be changed
         *
         * The PWM on/off ratio will be PWM_R:127.
         * The PWM_R argument byte has the format 0rrr rrrr, i.e. there are
         * 128 possible values between 0 and 127.
         */
        void pwm(uint8_t pwm_r, uint16_t pin_start, uint16_t pin_end = 0);

        /**
         * @brief PER command: Set pulse width modulation period for ALL pins
         *      simultaneously
         * @param per PWM period between 0 and 127
         *
         * The resulting PWM period will be clk_period/(per+1)*128.
         * This command is only available in PRO versions of the IOLinker chip.
         */
        void pwmPeriod(uint8_t per);

        /**
         * @brief CLR command: Clear pin links for pin range
         * @param pin_start First pin to update
         * @param pin_end Last pin to update, or 0 if only one is to be changed
         */
        void clearPinFunctions(uint16_t pin_start, uint16_t pin_end = 0);

        /**
         * @brief RST command: Reset volatile memory
         *
         * The software reset command will return the device to its default
         * power-on state and lose all current settings.
         */
        void reset(void);


        /** Chip chain scanning **/

        /**
         * @brief Try version retrieval for addresses starting at 1 and
         *      count up, return first address that lead to a reply. If
         *      all 127 addresses fail, return 0.
         * @return Returns the first node address that replied, or 0 if none
         *      worked.
         */
        uint16_t firstAddress(void);

        /**
         * @brief Try version retrieval for addresses starting at 'start' and
         *      counting up until it fails, return highest address that
         *      worked
         * @param start Address of the 'chain starting point'
         * @return Returns the length of the chai, or 0 if not even the first
         *      node replied.
         */
        uint16_t chainLength(uint8_t start = 1);

#define IOLINKER_CRCPOLY (0x89)
        uint8_t crc7(uint8_t *s, uint8_t len, uint8_t crc = 0);


#if defined(ARDUINO) || defined(WIRINGPI) /* Avoid private methods in PC
                                             unit test */
    private:
#endif

        /**
         * @brief Bit masks for the IOLinker protocol
         */
        enum bit_pos {
            IOLINKER_BITMASK_CMD_BIT = (1 << 7), /*!< Command bytes contain this bit */
            IOLINKER_BITMASK_RW_BIT = (1 << 6), /*!< If this bit is 1, the message is
                                                     of type 'read' */
            IOLINKER_BITMASK_BUF_BIT = (1 << 5), /*!< If this bit is 1, updated pin
                                                      states are supposed to be
                                                      buffered, rather than executed
                                                      directly */
            IOLINKER_BITMASK_CRC_BIT = (1 << 4), /*!< If this bit is 1, the message
                                                      ends with a CRC byte */
            IOLINKER_BITMASK_CMD = 0x4f, /*!< The part of the command byte that contains
                                              the actual command code; includes the RW
                                              bit as part of the command code */
            IOLINKER_BITMASK_DATA = 0x7f, /*!< The only bits that are set and may be
                                               set and are supposed to be used, in
                                               argument bytes */
            IOLINKER_BITMASK_PIN_ADDR = 0x7ff, /*!< I(nvert), V(irtual) and 10 pin
                                                    number bits make up the 12 bit pin
                                                    address */
            IOLINKER_REPLY_MAXMETA_BYTECOUNT = 0, /*!< Max meta byte count in replies,
                                                       address byte included */
            IOLINKER_REPLY_META_BYTECOUNT = 0, /*!< Meta byte count in replies that
                                                    is always there */
            IOLINKER_BITMASK_PIN_ADDRESS_VIRT = (1 << 11), /*!< Virtual pin bit */
            IOLINKER_BITMASK_PIN_ADDRESS_INV = (1 << 10), /*!< Invert pin state
                                                               bit */
        };

        enum virtual_pins {
            IOLINKER_VIRTUAL_CLK = (0x01 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK2 = (0x02 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK4 = (0x03 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK8 = (0x04 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK16 = (0x05 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK32 = (0x06 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK64 = (0x07 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK128 = (0x08 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
            IOLINKER_VIRTUAL_CLK256 = (0x09 | IOLINKER_BITMASK_PIN_ADDRESS_VIRT),
        };

        uint8_t __crc = 0; /*!< CRC of the message currently being
                                constructed */
        uint8_t target_addr = IOLINKER_TARGET_ALL; /*!< Current target address */
        uint8_t cmdbyte = IOLINKER_BITMASK_CMD_BIT; /*!< Current command byte */
        
        /** Protocol format **/
        /* Byte order:
           Command, address, arg1, ..., CRC

           The address byte is left out in I2C mode, and the CRC byte is left
           out if CRC is not active.
        */

        /**
         * @brief Extract state of IOLINKER_BITMASK_CMD_BIT from byte
         */

        inline bool cmdBitOn(uint8_t b)
        {
            return ((b & IOLINKER_BITMASK_CMD_BIT) != 0);
        }
        
        /**
         * @brief Extract state of BITMASK_RW_BIT from byte
         */
        inline bool rwBitOn(uint8_t b)
        {
            return ((b & IOLINKER_BITMASK_RW_BIT) != 0);
        }
        
        /**
         * @brief Extract state of BITMASK_BUF_BIT from byte
         */
        inline bool bufBitOn(uint8_t b)
        {
            return ((b & IOLINKER_BITMASK_BUF_BIT) != 0);
        }
        
        /**
         * @brief Extract state of BITMASK_CRC_BIT from byte
         */
        inline bool crcBitOn(uint8_t b)
        {
            return ((b & IOLINKER_BITMASK_CRC_BIT) != 0);
        }
        
        /**
         * @brief Extract command code from byte
         */
        inline uint8_t commandCode(uint8_t b)
        {
            return (b & IOLINKER_BITMASK_CMD);
        }
        
        /**
         * @brief Extract argument code from byte
         */
        inline uint8_t argData(uint8_t b)
        {
            return (b & IOLINKER_BITMASK_DATA);
        }

        /**
         * @brief Return the command byte in the string buffer
         * @param buf String buffer containing one entire message
         * @param len String length
         * @return The command byte
         */
        inline uint8_t cmdByte(uint8_t *buf, uint8_t len)
        {
            if (len < 1 || ((buf[0] & IOLINKER_BITMASK_CMD_BIT) !=
                        IOLINKER_BITMASK_CMD_BIT)) {
                return 0;
            }
            return buf[0];
        }
        
        /**
         * @brief Determine the number of address bytes for the current
         *      communication interface in reply messages
         * @return Number of address bytes that is supposed to be contained
         *      in every message
         */
        inline uint8_t addrByteCount(void)
        {
            return 0;
            //return ((interface_mode == IOLINKER_I2C) ? 0 : 1);
        }
            
        /**
         * @brief Determine the number of optional meta bytes for the current
         *      communication settings
         * @return Number of optional meta bytes bytes that are required with
         *      current configuration
         */
        inline uint8_t optionalMetaByteCount(void)
        {
            return 0;
            //return addrByteCount() + ((crcBitOn(cmdbyte)) ? 1 : 0);
        }
        
        /**
         * @brief Return the address byte in the string buffer
         * @param buf String buffer containing one entire message
         * @param len String length
         * @return The address byte
         */
        inline uint8_t addrByte(uint8_t *buf, uint8_t len)
        {
            if (addrByteCount() == 0) {
                /* If no address byte is available within the message, we
                   return the one stored in buffer */
                return target_addr;
            }

            if (len < 1) {
                return 0;
            }
            return buf[1];
        }
        
        /**
         * @brief Return a byte argument from string buffer
         * @param buf String buffer containing one entire message
         * @param len String length
         * @param i Argument index
         * @return The argument byte
         */
        inline uint8_t argByte(uint8_t *buf, uint8_t len, uint8_t i)
        {
            if (len < (addrByteCount() + i)) {
                return 0;
            }
            return buf[addrByteCount() + i];
        }
        
        /**
         * @brief Return a word argument from string buffer
         * @param buf String buffer containing one entire message
         * @param len String length
         * @param i Argument index
         * @return The argument byte
         */
        inline uint8_t argWord(uint8_t *buf, uint8_t len, uint8_t i)
        {
            if (len < (1 + addrByteCount() + i + 1)) {
                return 0;
            }
            return (argByte(buf, len, i) << 7 | argByte(buf, len, i + 1));
        }
        
        /**
         * @brief Return the CRC byte from string buffer
         * @param buf String buffer containing one entire message
         * @param len String length
         * @return The CRC byte
         */
        inline uint8_t crcByte(uint8_t *buf, uint8_t len)
        {
            if (len < 2) {
                return 0;
            }
            return buf[len - 1];
        }
        
        inline uint16_t virtual_pin(uint16_t pin_address)
        {
            return (pin_address | IOLinker::IOLINKER_BITMASK_PIN_ADDRESS_VIRT);
        }

        inline uint16_t invert(uint16_t pin_address)
        {
            return (pin_address | IOLinker::IOLINKER_BITMASK_PIN_ADDRESS_INV);
        }


        /**
         * @brief Command codes
         */
        typedef enum cmd_t {
            IOLINKER_CMD_VER = 0x01 | IOLINKER_BITMASK_RW_BIT,
            IOLINKER_CMD_TYP = 0x02,
            IOLINKER_CMD_REA = 0x07 | IOLINKER_BITMASK_RW_BIT,
            IOLINKER_CMD_SET = 0x03,
            IOLINKER_CMD_SYN = 0x08,
            IOLINKER_CMD_TRG = 0x09,
            IOLINKER_CMD_LNK = 0x04,
            IOLINKER_CMD_PWM = 0x05,
            IOLINKER_CMD_PER = 0x06,
            IOLINKER_CMD_CLR = 0x0a,
            IOLINKER_CMD_RST = 0x0f,
        } cmd_t;

        /**
         * @brief Extract pin address
         */
        inline uint16_t pin_addr(uint16_t pin)
        {
            return (pin & IOLINKER_BITMASK_PIN_ADDR);
        }

        /**
         * @brief Determine the distance of two pin addresses
         */
        inline uint8_t pin_distance(uint16_t pin1, uint16_t pin2)
        {
            if (pin2 == 0) {
                return 0;
            }

            pin1 = pin_addr(pin1);
            pin2 = pin_addr(pin2);
            if (pin1 >= pin2) {
                return pin1 - pin2;
            }
            return pin2 - pin1;
        }
        
        /**
         * @brief Finish message and read reply of the given max length into
         *      the buffer and verify CRC if applicable.
         * @param buf String buffer to write into, with at least
         *       optionalMetaByteCount() + REPLY_META_BYTECOUNT bytes
         * @param len String length
         * @return Returns the length of the reply that has been received
         */
        uint8_t finishAndReadReply(uint8_t *buf = NULL, uint8_t len = 0);

        /* Reset CRC and write out command +
                                     address byte, if applicable */
        void writeCmd(cmd_t cmd);
        
        /**
         * @brief Write out additional message part
         */
        void writeMsg(uint8_t *s, uint8_t len);
        
        /**
         * @brief Write out CRC if applicable
         */
        inline bool writeCRC(void)
        {
            if (crcBitOn(cmdbyte)) {
                //printf("CRC 0x%x\n", __crc);
                writeMsg(&__crc, 1);
            }
        }
};

#endif /* __IOLINKER_H__ */

