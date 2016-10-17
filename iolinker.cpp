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

#include "iolinker.h"
#include <string.h>

#ifdef __PC
#include <unistd.h>
#endif


#if defined(WIRINGPI) || defined(__PC)
void iolinker::beginSerial(const char *dev)
{
    interface_mode = UART;
    //interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE_WIRINGPI);
    interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE);
}
#endif

#ifdef WIRINGPI
void iolinker::beginSPI(uint8_t channel)
{
    interface_mode = SPI;
    interface_fd = wiringPiSPISetup(channel, __IOLINKER_SPI_CLK);
}

void iolinker::beginI2C(void)
{
    interface_mode = I2C;
    interface_fd = wiringPiI2CSetup(target_addr);
}

#elif ARDUINO
#endif


/** Messages **/
        
uint16_t iolinker::version(void)
{
    uint8_t buf[2];
    writeCmd(CMD_VER);
    writeCRC();
    if (!readReply(buf, sizeof(buf))) {
        return 0;
    }
    return (buf[0] << 8 | buf[1]);
}

void iolinker::setPinType(pin_types type, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end),
            argData(type) };
    writeCmd(CMD_TYP);
    writeMsg(buf, sizeof(buf));
    writeCRC();
    readReply();
}

bool iolinker::readInput(uint16_t pin)
{
    uint8_t tx[] = { argData(pin >> 7), argData(pin), 0, 0 };
    uint8_t buf;
    writeCmd(CMD_REA);
    writeMsg(tx, sizeof(tx));
    writeCRC();
    if (!readReply(&buf, 1)) {
        return 0;
    }
    return ((buf >> 7) == 1);
}

void iolinker::readInput(uint8_t *s, uint8_t len, uint16_t pin_start,
        uint16_t pin_end)
{
    uint8_t bytecount = (uint8_t)(
            (pin_distance(pin_start, pin_end) + 0.5) / 7);
    if (bytecount < len) {
        len = bytecount;
    }

    uint8_t tx[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(CMD_REA);
    writeMsg(tx, sizeof(tx));
    writeCRC();

    if (!readReply(s, len)) {
        return;
    }

    uint8_t offset = 1, j = 0, lastbyte = 0;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = (s[j] << offset);
        byte |= (s[j + 1] >> (7 - offset));

        s[j] = byte;

        if (++offset >= 8) {
            offset = 1;
        } else {
            j++;
        }
    }
}

void iolinker::setOutput(bool state, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t bytecount = (uint8_t)(
            (pin_distance(pin_start, pin_end) + 0.5) / 7);
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(CMD_SET);
    writeMsg(buf, sizeof(buf));
    
    uint8_t byte = ((state) ? 0xff : 0x00);
    for (uint8_t i = 0; i < bytecount; i++) {
        writeMsg(&byte, 1);
    }
    
    writeCRC();
    readReply();
}

void iolinker::setOutput(uint8_t *s, uint8_t len, uint16_t pin_start,
        uint16_t pin_end)
{
    uint8_t bytecount = max(1,
            (uint8_t)((pin_distance(pin_start, pin_end) + 0.5) / 7));
    uint8_t buf[4] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(CMD_SET);
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
    readReply();
}

void iolinker::syncOutputsToBuffer(void)
{
    writeCmd(CMD_SYN);
    writeCRC();
    readReply();
}

void iolinker::syncBufferToOutputs(void)
{
    writeCmd(CMD_TRG);
    writeCRC();
    readReply();
}

void iolinker::link(uint16_t target_pin, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end),
            argData(target_pin >> 7), argData(target_pin), };
    writeCmd(CMD_PWM);
    writeMsg(buf, sizeof(buf));
    writeCRC();
    readReply();
}

void iolinker::pwm(uint8_t pwm_r, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end),
            argData(pwm_r), };
    writeCmd(CMD_PWM);
    writeMsg(buf, sizeof(buf));
    writeCRC();
    readReply();
}

void iolinker::pwmPeriod(uint8_t per)
{
    per = argData(per);
    writeCmd(CMD_PER);
    writeMsg(&per, 1);
    writeCRC();
    readReply();
}

void iolinker::reset(void)
{
    writeCmd(CMD_RST);
    writeCRC();
    readReply();
}


/** Chip chain scanning **/

uint16_t iolinker::firstAddress(void)
{
    uint8_t addr = 0;
    
    do {
        targetAddress(++addr);
    } while (!available() && addr < TARGET_MAX);

    return ((addr >= TARGET_MAX) ? 0 : addr);
}

uint16_t iolinker::chainLength(uint8_t start)
{
    uint8_t len = 0;
    
    do {
        targetAddress(start + len);
        len++;
    } while (available());
    
    return len - 1;
}

bool iolinker::readReply(uint8_t *s, uint8_t len)
{
    uint8_t i = 0;

#ifdef WIRINGPI
    if (interface_fd == -1) {
        status = ERROR_INTERFACE;
        return false;
    }
    
    read(interface_fd, s, len);
#elif defined(ARDUINO)
    if (interface_mode == I2C) {
        Wire.requestFrom(target_addr, len);
        for (; i < len && Wire.available(); i++) {
            s[i] = (uint8_t)Wire.read();
        }
    } else if (interface_mode == SPI) {
#if 0 // Delay?
        resetSS();
        delay(1);
        setSS();
#endif

        SPI.beginTransaction(__IOlINKER_SPI_SETTINGS); 
        SPI.transfer(s, len);
        SPI.endTransaction();
        resetSS();
    } else if (interface_mode == UART) {
        for (; i < len; i++) {
            s[i] = interface_stream->read();
        }
    }
    
#elif defined(__PC)
    interface_buf = interface_buf_reset;
    i = interface_testfunc(s, len);
    /* Reply is in '*s', and is of length 'size' */
#endif

    /* Verify length of return message */
    if (i < len) {
        status = ERROR_NOREPLY;
        return false;
    }

    /* Verify CRC */
    // TODO
    if (0) {
        status = ERROR_CRC;
        return false;
    }

    status = STATUS_SUCCESS;
    return true;
}

void iolinker::writeMsg(uint8_t *s, uint8_t len)
{
#ifdef WIRINGPI
    /* Should work for UART, SPI and I2C alike. */
    if (interface_mode == I2C ||
            interface_mode == SPI ||
            interface_mode == UART) {
        writeMsg(interface_fd, s, len);
    }
#elif defined(ARDUINO)
    for (uint8_t i = 0; i < len; i++) {
        if (interface_mode == I2C) {
            Wire.writeMsg(s[i]);
        } else if (interface_mode == SPI) {
            SPI.transfer(s[i]);
        } else if (interface_mode == UART) {
            interface_stream->writeMsg(s[i]);
        }
    }
#else
    if (interface_mode == UART) {
        write(interface_fd, s, len);
    }
#endif

    else if (interface_mode == INTERFACE_UNCLEAR) {
        if ((interface_buf+len) <= interface_buf_end) {
            strncpy((char *)interface_buf, (const char *)s, (size_t)len);
            interface_buf += len;
        }
    }
}

void iolinker::writeCmd(cmd_t cmd)
{
    __crc = 0;
#ifdef __PC
    interface_buf = interface_buf_reset;
#endif

    cmdbyte &= ~(BITMASK_CMD);
    cmdbyte |= cmd;

#ifdef WIRINGPI
    /* Nothing to do on Raspberry */
#elif defined(ARDUINO)
    if (interface_mode == SPI) {
        setSS();
        SPI.beginTransaction(__IOlINKER_SPI_SETTINGS); 
    } else if (interface_mode == I2C) {
        Wire.beginTransmission(target_addr);
    } else if (interface_mode == UART) {
        /* Nothing to prepare for UART transmission */
    }
#else
    /* Nothing to prepare for PC UART transmission */
#endif

    writeMsg(&cmdbyte, 1);
}

