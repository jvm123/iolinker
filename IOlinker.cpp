/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/iolinker
 */

/**
 * @file iolinker.cpp
 * @author Julian von Mendel
 * @brief iolinker class for Arduino, Raspberry Pi and PC unit tests
 **/

#include "IOlinker.h"
#include <string.h>

#ifdef __PC
#include <unistd.h>
#elif defined(ARDUINO)
#include <Stream.h>
#include <SPI.h>
#endif

#if defined(WIRINGPI) || defined(__PC)
void IOlinker::beginSerial(const char *dev)
{
    interface_mode = IOLINKER_UART;
    //interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE_WIRINGPI);
    interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE);
}
#endif

#ifdef WIRINGPI
void IOlinker::beginSPI(uint8_t channel)
{
    interface_mode = IOLINKER_SPI;
    interface_fd = wiringPiSPISetup(channel, __IOLINKER_SPI_CLK);
}

void IOlinker::beginI2C(void)
{
    interface_mode = IOLINKER_I2C;
    interface_fd = wiringPiI2CSetup(target_addr);
}

#elif defined(ARDUINO)
#endif


/** Messages **/
        
uint16_t IOlinker::version(void)
{
    uint8_t buf[2 + IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeCmd(IOLINKER_CMD_VER);
    writeCRC();
    if (!readReply(buf, 2 + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT)) {
        return 0;
    }
    return (argByte(buf, sizeof(buf), 1) << 8 | argByte(buf, sizeof(buf), 2));
}

void IOlinker::setPinType(pin_types type, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf_reply[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end),
            argData(type) };
    writeCmd(IOLINKER_CMD_TYP);
    writeMsg(buf, sizeof(buf));
    writeCRC();
    readReply(buf_reply, optionalMetaByteCount() +
            IOLINKER_REPLY_META_BYTECOUNT);
}

bool IOlinker::readInput(uint16_t pin)
{
    uint8_t tx[] = { argData(pin >> 7), argData(pin), 0, 0 };
    uint8_t buf[1 + IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeCmd(IOLINKER_CMD_REA);
    writeMsg(tx, sizeof(tx));
    writeCRC();
    if (!readReply(buf, 1 + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT)) {
        return 0;
    }
    return ((argByte(buf, sizeof(buf), 1) >> 6) == 1);
}

void IOlinker::readInput(uint8_t *s, uint8_t len, uint16_t pin_start,
        uint16_t pin_end)
{
    if (pin_end < pin_start) {
        pin_end = pin_start;
    }

    uint8_t bytecount = (uint8_t)(
            (pin_distance(pin_start, pin_end) + 1.5) / 7);
    if (bytecount < len) {
        len = bytecount;
    }

    uint8_t tx[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(IOLINKER_CMD_REA);
    writeMsg(tx, sizeof(tx));
    writeCRC();

    if (!readReply(s, len + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT)) {
        return;
    }

    uint8_t offset = 1, j = 0, lastbyte = 0;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = (argByte(s, sizeof(s), 1 + j) << offset);
        byte |= (argByte(s, sizeof(s), 2 + j) >> (7 - offset));

        s[j] = byte;

        if (++offset >= 8) {
            offset = 1;
        } else {
            j++;
        }
    }
}

void IOlinker::setOutput(bool state, uint16_t pin_start, uint16_t pin_end)
{
    if (pin_end < pin_start) {
        pin_end = pin_start;
    }

    uint8_t buf_reply[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    uint8_t bytecount = (uint8_t)(
            (pin_distance(pin_start, pin_end) + 1.5) / 7);
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(IOLINKER_CMD_SET);
    writeMsg(buf, sizeof(buf));
    
    uint8_t byte = ((state) ? 0xff : 0x00);
    for (uint8_t i = 0; i < bytecount; i++) {
        writeMsg(&byte, 1);
    }
    
    writeCRC();
    readReply(buf_reply, optionalMetaByteCount() +
            IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::setOutput(uint8_t *s, uint8_t len, uint16_t pin_start,
        uint16_t pin_end)
{
    if (pin_end < pin_start) {
        pin_end = pin_start;
    }

    uint8_t buf_reply[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    uint8_t bytecount = max(1,
            (uint8_t)((pin_distance(pin_start, pin_end) + 1.5) / 7));
    uint8_t buf[4] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(IOLINKER_CMD_SET);
    writeMsg(buf, sizeof(buf));
    
    uint8_t offset = 1, j = 0;

    for (uint8_t i = 0; i < bytecount; i++) {
        uint8_t byte = (s[j] >> offset);
        
        if (j > 0) {
            byte |= (s[j-1] << (7 - offset));
        }

        writeMsg(&byte, 1);

        if (++offset >= 8) {
            offset = 1;
        } else {
            j++;
        }
    }

    writeCRC();
    readReply(buf_reply, optionalMetaByteCount() +
            IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::syncOutputsToBuffer(void)
{
    uint8_t buf[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeCmd(IOLINKER_CMD_SYN);
    writeCRC();
    readReply(buf, optionalMetaByteCount() + IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::syncBufferToOutputs(void)
{
    uint8_t buf[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeCmd(IOLINKER_CMD_TRG);
    writeCRC();
    readReply(buf, optionalMetaByteCount() + IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::link(uint16_t target_pin, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf_reply[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end),
            argData(target_pin >> 7), argData(target_pin), };
    writeCmd(IOLINKER_CMD_LNK);
    writeMsg(buf, sizeof(buf));
    writeCRC();
    readReply(buf_reply, optionalMetaByteCount() +
            IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::pwm(uint8_t pwm_r, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf_reply[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end),
            argData(pwm_r), };
    writeCmd(IOLINKER_CMD_PWM);
    writeMsg(buf, sizeof(buf));
    writeCRC();
    readReply(buf_reply, optionalMetaByteCount() +
            IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::pwmPeriod(uint8_t per)
{
    uint8_t buf[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    per = argData(per);
    writeCmd(IOLINKER_CMD_PER);
    writeMsg(&per, 1);
    writeCRC();
    readReply(buf, optionalMetaByteCount() + IOLINKER_REPLY_META_BYTECOUNT);
}

void IOlinker::reset(void)
{
    uint8_t buf[IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeCmd(IOLINKER_CMD_RST);
    writeCRC();
    readReply(buf, optionalMetaByteCount() + IOLINKER_REPLY_META_BYTECOUNT);
}


/** Chip chain scanning **/

uint16_t IOlinker::firstAddress(void)
{
    uint8_t addr = 0;
    
    do {
        targetAddress(++addr);
    } while (!available() && addr < IOLINKER_TARGET_MAX);

    return ((addr >= IOLINKER_TARGET_MAX) ? 0 : addr);
}

uint16_t IOlinker::chainLength(uint8_t start)
{
    uint8_t len = 0;
    
    do {
        targetAddress(start + len);
        len++;
    } while (available());
    
    return len - 1;
}


uint8_t IOlinker::crc7(uint8_t *s, uint8_t len, uint8_t crc)
{ /* Source: http://www.microchip.com/forums/FindPost/100156 */

    for (uint8_t i = 0; i < len; i++, s++) {
        uint8_t c = *s;

        for (uint8_t ibit = 0; ibit < 8; ibit++) {
            crc = crc << 1;
            if ((c ^ crc) & 0x80) {
                crc = crc ^ 0x09;
            }
            c = c << 1;
        }

        crc = crc & 0x7F;
    }
    return crc;
}

bool IOlinker::readReply(uint8_t *s, uint8_t len)
{
    uint8_t i = 0;

#if defined(WIRINGPI) || defined(__PC)
    if (interface_mode == IOLINKER_I2C || interface_mode == IOLINKER_SPI ||
            interface_mode == IOLINKER_UART) {
        if (interface_fd == -1) {
            status = IOLINKER_ERROR_INTERFACE;
            return false;
        }
        
        read(interface_fd, s, len);
    }
#elif defined(ARDUINO)
    if (interface_mode == IOLINKER_I2C) {
        Wire.requestFrom(target_addr, len);
        for (; i < len && Wire.available(); i++) {
            s[i] = (uint8_t)Wire.read();
        }
    } else if (interface_mode == IOLINKER_SPI) {
#if 0 // Delay?
        digitalWrite(__IOLINKER_SPI_CS, HIGH); // unselect
        delay(1);
        digitalWrite(__IOLINKER_SPI_CS, LOW); // select
#endif

        SPI.beginTransaction(__IOlINKER_SPI_SETTINGS); 
        SPI.transfer(s, len);
        SPI.endTransaction();
        digitalWrite(__IOLINKER_SPI_CS, HIGH); // unselect
    } else if (interface_mode == IOLINKER_UART) {
        for (; i < len; i++) {
            s[i] = interface_stream->read();
        }
    }
#endif
    else if (interface_mode == IOLINKER_INTERFACE_CALLBACK) {
        i = interface_testfunc(interface_buf_reset,
                interface_buf - interface_buf_reset);
        /* Reply is in '*s', and is of length 'size' */
        interface_buf = interface_buf_reset;
        strncpy((char *)s, (const char *)interface_buf, i);
    }

    /* Verify length of return message */
    if (i < len || i < (2 + optionalMetaByteCount())) {
        status = IOLINKER_ERROR_NOREPLY;
        return false;
    }

    /* Verify CRC */
    if (crcBitOn(cmdByte(s, len))) {
        s[i - 1] <<= 1;
        
        if (crc7(s, i) != 0) {
            status = IOLINKER_ERROR_CRC;
            return false;
        }
    }

    status = (IOlinker::status_code)s[1 + addrByteCount()];
    return true;
}

void IOlinker::writeMsg(uint8_t *s, uint8_t len)
{
#ifdef WIRINGPI
    /* Should work for UART, SPI and I2C alike. */
    if (interface_mode == IOLINKER_I2C ||
            interface_mode == IOLINKER_SPI ||
            interface_mode == IOLINKER_UART) {
        if (interface_fd == -1) {
            status = IOLINKER_ERROR_INTERFACE;
            return;
        }
        
        writeMsg(interface_fd, s, len);
    }
#elif defined(ARDUINO)
    for (uint8_t i = 0; i < len; i++) {
        if (interface_mode == IOLINKER_I2C) {
            Wire.write(s[i]);
        } else if (interface_mode == IOLINKER_SPI) {
            SPI.transfer(s[i]);
        } else if (interface_mode == IOLINKER_UART) {
            interface_stream->write(s[i]);
        }
    }
#else
    if (interface_mode == IOLINKER_UART) {
        if (interface_fd == -1) {
            status = IOLINKER_ERROR_INTERFACE;
            return;
        }
        
        write(interface_fd, s, len);
    }
#endif

    if (interface_mode == IOLINKER_INTERFACE_CALLBACK) {
        if ((interface_buf + len) <= interface_buf_end) {
            strncpy((char *)interface_buf, (const char *)s, (size_t)len);
            interface_buf += len;
        }
    }

    __crc = crc7(s, len, __crc);
}

void IOlinker::writeCmd(cmd_t cmd)
{
    __crc = 0;
#ifdef __PC
    interface_buf = interface_buf_reset;
#endif

    cmdbyte &= ~(IOLINKER_BITMASK_CMD);
    cmdbyte |= cmd;

#ifdef WIRINGPI
    /* Nothing to do on Raspberry */
#elif defined(ARDUINO)
    if (interface_mode == IOLINKER_SPI) {
        digitalWrite(__IOLINKER_SPI_CS, LOW); // select
        SPI.beginTransaction(__IOlINKER_SPI_SETTINGS); 
    } else if (interface_mode == IOLINKER_I2C) {
        Wire.beginTransmission(target_addr);
    } else if (interface_mode == IOLINKER_UART) {
        /* Nothing to prepare for UART transmission */
    }
#else
    /* Nothing to prepare for PC UART transmission */
#endif

    writeMsg(&cmdbyte, 1);
    writeMsg(&target_addr, 1);
}

