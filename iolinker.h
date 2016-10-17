/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/iolinker
 * iolinker class header
 **/

/* TODO: Doxygen

Inspiration:
    cd /home/jvm/bin/progs/arduino-1.7.11-linux32/libraries/

    Serial/Stream https://github.com/andrewrapp/xbee-arduino/blob/master/XBee.cpp
    SPI Ethernet/src/utility/w5100.h
    I2C LuckyShield/src/lib/BME280.cpp https://github.com/adafruit/Adafruit-BMP085-Library/blob/master/Adafruit_BMP085.cpp

    Raspberry: http://wiringpi.com/reference/serial-library/ https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial

Arduino self-test example includes unit test (on device), chain scan and serial debugging output.
Raspberry self-test example includes unit test, chain scan.
PC Unit test runs unit test only.
*/

#ifndef __IOLINKER_H__
#define __IOLINKER_H__

#include <stdlib.h>
#include <stdint.h>
#define max(a,b) (a>b?a:b)

#ifdef WIRINGPI
#include <wiringSerial.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#elif defined(ARDUINO)
#include "Arduino.h"
#elif !defined(__PC)
#define __PC
#endif

#define __IOLINKER_DEBUG (1) /* Activate debugging output */


class iolinker {
    public:
#define __IOLINKER_BAUDRATE (115200)

#ifdef WIRINGPI
        void beginSerial(unsigned char *dev); /* Setup serial interface */
#define __IOLINKER_SPI_CLK (32000000UL)
        
        void beginSPI(uint8_t channel);  /* Setup SPI master */
        
        void beginI2C(void); /* Setup  I2C master */

#elif defined(ARDUINO)
        /* Setup serial interface */
        inline void beginStream(Stream &stream)
        {
            interface_mode = UART;
            interface_stream = &stream;
        }

#define __IOLINKER_SPI_CS (10)        
        /* Setup SPI master */
        inline void beginSPI(void)
        {
            interface_mode = SPI;
            SPI.begin();
            initSS();
            SPI.setClockDivider(SPI_CS, 21);
            SPI.setDataMode(SPI_CS, SPI_MODE0);
        }
        
        /* Setup  I2C master */
        inline void beginI2C(void)
        {
            interface_mode = I2C;
            Wire.begin();
        }

#else
        void beginSerial(unsigned char *dev); /* Setup serial interface */
#endif

        typedef uint8_t (*testfunc_t)(unsigned char *s, uint8_t len);
        
        /* Messages are written into the string pointer *buf, until at message end the provided callback function is run, that provides its reply in the same buffer *buf. Can be used for unit testing. */
        inline void beginTest(testfunc_t testfunc, uint8_t *buf, uint8_t size)
        {
            interface_mode = INTERFACE_UNCLEAR;
            interface_testfunc = testfunc;
            interface_buf = buf;
            interface_buf_reset = buf;
            interface_buf_end = buf + size;
        }


        /** Message preparation **/

        typedef enum target_address {
            TARGET_ALL = 0x00,
            TARGET_FIRST = 0x01,
            TARGET_MAX = 0x7f,
        } target_address;

        /* Choose slave address */
        inline void targetAddress(uint8_t addr)
        {
#ifdef WIRINGPI
            if (interface_mode == I2C && target_addr != addr) {
                interface_fd = wiringPiI2CSetup(addr);
            }
#endif

            target_addr = addr;
        }

        /* Turn buffering on or off for following messages */
        inline void buffer(bool active)
        {
            if (active) {
                cmdbyte |= BITMASK_BUF_BIT;
            } else {
                cmdbyte &= ~(BITMASK_BUF_BIT);
            }
        }

        /* Turn CRC error detection on or off for following messages */
        inline void crc(bool active)
        {
            if (active) {
                cmdbyte |= BITMASK_CRC_BIT;
            } else {
                cmdbyte &= ~(BITMASK_CRC_BIT);
            }
        }

        /** Status codes **/

        typedef enum status_code {
            STATUS_UNDEFINED = 0x00,
            STATUS_SUCCESS = 0x01,
            ERROR = 0x02,
            ERROR_ARGCOUNT = 0x03,
            ERROR_PINNUM = 0x04,
            ERROR_INTERFACE = 0xfc,
            ERROR_NOREPLY = 0xfd,
            ERROR_CRC = 0xff,
        } status_code;
        
        /* Return status code of last reply */
        inline status_code statusCode(void)
        {
            return status;
        }

        /* Check chip availability */
        inline bool available(void)
        {
            version();
            return (statusCode() == STATUS_SUCCESS);
        }


        /** Messages **/

        /* VER command: Retrieve chip version */
        uint8_t version(void);

        /* Derive pin count from chip version byte */
        inline uint16_t pinCount(uint8_t version)
        {
            switch (version & 0x0f) {
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

        /* Derive pro version status from chip version byte */
        inline bool isProVersion(uint8_t version)
        {
            return ((version & (1 << 7)) == 1);
        }

        typedef enum pin_types {
            INPUT = 0x00,
            PULLDOWN = 0x01,
            PULLUP = 0x02,
            OUTPUT = 0x03,
        } pin_types;
        void setPinType(pin_types type, uint16_t pin_start, uint16_t pin_end = 0); /* TYP command: Set pin type */

        bool readInput(uint16_t pin); /* REA command: Read input state */

        void readInput(uint8_t *s, uint8_t len, uint16_t pin_start, uint16_t pin_end = 0); /* REA command: Read input pin range and store value in buffer */

        void setOutput(bool state, uint16_t pin_start, uint16_t pin_end = 0); /* SET command: Set output pin states */

        void setOutput(uint8_t *s, uint8_t len, uint16_t pin_start, uint16_t pin_end = 0); /* SET command: Set output pin states */

        void syncOutputsToBuffer(void); /* SYN command */

        void syncBufferToOutputs(void); /* TRG command */

        void link(uint16_t target_pin, uint16_t pin_start, uint16_t pin_end = 0); /* LNK command: Link pins */

        void pwm(uint8_t pwm_r, uint16_t pin_start, uint16_t pin_end = 0); /* PWM command */

        void pwmPeriod(uint8_t per); /* PER command */

        void reset(void); /* RST command */


        /** Chip chain scanning **/

        uint16_t firstAddress(void); /* Try version retrieval for addresses starting at 1 and count up, return first address that lead to a reply. If all 127 addresses fail, return 0. */

        uint16_t chainLength(uint8_t start = 1); /* Try version retrieval for addresses starting at 'start' and counting up until it fails, return highest address that worked */ 

#if defined(ARDUINO) || defined(WIRINGPI) /* Avoid private methods in PC unit test */
    private:
#endif

        enum bit_pos {
            BITMASK_CMD_BIT = (1 << 7),
            BITMASK_RW_BIT = (1 << 6),
            BITMASK_BUF_BIT = (1 << 5),
            BITMASK_CRC_BIT = (1 << 4),
            BITMASK_CMD = 0x4f, /*< Includes the RW bit as part of the command
                                    code */
            BITMASK_DATA = 0x7f,
            BITMASK_PIN_ADDR = 0x7ff,
        };

        status_code status = STATUS_UNDEFINED;
        uint8_t __crc = 0;
        uint8_t target_addr = TARGET_ALL; /*< Current target address */
        uint8_t cmdbyte = BITMASK_CMD_BIT;
        
        enum interface_mode {
            UART = 0,
            SPI,
            I2C,
            INTERFACE_UNCLEAR,
        } interface_mode;
        
#ifndef ARDUINO
        testfunc_t interface_testfunc;
        unsigned char *interface_buf, *interface_buf_reset,
                      *interface_buf_end;
#else
        Stream *interface_stream;
#endif

#if defined(WIRINGPI) || defined(__PC)
        int interface_fd;
#endif

        /** Protocol format **/
        /* Byte order:
           Command, address, arg1, ..., CRC

           The address byte is left out in I2C mode, and the CRC byte is left
           out if CRC is not active.
        */

        inline uint8_t cmdByte(uint8_t *buf, uint8_t len)
        {
            if (len < 1) {
                return 0;
            }
            return buf[0];
        }
        inline uint8_t addrByte(uint8_t *buf, uint8_t len)
        {
            if (interface_mode == I2C) {
                /* In I2C mode, the address byte is not part of the message
                   content. */
                return target_addr;
            }

            if (len < 2) {
                return 0;
            }
            return buf[1];
        }
        inline uint8_t addrByteCount(void)
        {
            return ((interface_mode == I2C) ? 0 : 1);
        }
        inline uint8_t argByte(uint8_t *buf, uint8_t len, uint8_t i)
        {
            if (len < (1 + addrByteCount() + i)) {
                return 0;
            }
            return buf[1 + addrByteCount() + i];
        }
        inline uint8_t crcByte(uint8_t *buf, uint8_t len)
        {
            if (len < 2) {
                return 0;
            }
            return buf[len - 2];
        }

        inline bool cmdBitOn(uint8_t b)
        {
            return ((b & BITMASK_CMD_BIT) != 0);
        }
        inline bool rwBitOn(uint8_t b)
        {
            return ((b & BITMASK_RW_BIT) != 0);
        }
        inline bool bufBitOn(uint8_t b)
        {
            return ((b & BITMASK_BUF_BIT) != 0);
        }
        inline bool crcBitOn(uint8_t b)
        {
            return ((b & BITMASK_CRC_BIT) != 0);
        }
        inline uint8_t commandCode(uint8_t b)
        {
            return (b & BITMASK_CMD);
        }
        inline uint8_t argData(uint8_t b)
        {
            return (b & BITMASK_DATA);
        }

        typedef enum cmd_t {
            CMD_VER = 0x01,
            CMD_TYP = 0x02 | BITMASK_RW_BIT,
            CMD_REA = 0x07,
            CMD_SET = 0x03 | BITMASK_RW_BIT,
            CMD_SYN = 0x08 | BITMASK_RW_BIT,
            CMD_TRG = 0x09 | BITMASK_RW_BIT,
            CMD_LNK = 0x04 | BITMASK_RW_BIT,
            CMD_PWM = 0x05 | BITMASK_RW_BIT,
            CMD_PER = 0x06 | BITMASK_RW_BIT,
            CMD_RST = 0x0f | BITMASK_RW_BIT,
        } cmd_t;

        inline uint16_t pin_addr(uint16_t pin)
        {
            return (pin & BITMASK_PIN_ADDR);
        }
        inline uint8_t pin_distance(uint16_t pin1, uint16_t pin2)
        {
            pin1 = pin_addr(pin1);
            pin2 = pin_addr(pin2);
            if (pin1 >= pin2) {
                return pin1 - pin2;
            }
            return pin2 - pin1;
        }
        
        bool readReply(uint8_t *s = NULL, uint8_t len = 0); /* Read reply of the given max length into the buffer, verify CRC if applicable, and save status code for the statusCode() function. Return false and set status code to ERROR_CRC on CRC failure or no received reply, otherwise return true. */

        void writeCmd(cmd_t cmd); /* Reset CRC and write out command +
                                     address byte, if applicable */
        void writeMsg(uint8_t *s, uint8_t len); /* Write out additional message
                                                part */
        
        /* Write out CRC if applicable */
        inline bool writeCRC(void)
        {
            writeMsg(&__crc, 1);
        }
};

#endif /* __IOLINKER_H__ */

