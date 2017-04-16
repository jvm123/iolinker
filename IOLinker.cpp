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

#include "IOLinker.h"
#include <string.h>

#ifdef WIRINGPI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#endif

#if defined(__PC) || defined(WIRINGPI)
#include <unistd.h>
#elif defined(ARDUINO)
#include <Stream.h>
#include <SPI.h>
#endif

#if defined(WIRINGPI) || defined(__PC)
void IOLinker::beginSerial(const char *dev)
{
    interface_mode = IOLINKER_UART;
    //interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE_WIRINGPI);
    interface_fd = serialOpen(dev, IOLINKER_BAUDRATE);
}
#endif

#ifdef WIRINGPI
void IOLinker::beginSPI(uint8_t channel)
{
    interface_mode = IOLINKER_SPI;
    interface_fd = wiringPiSPISetup(channel, __IOLINKER_SPI_CLK);
}

void IOLinker::beginI2C(void)
{
    interface_mode = IOLINKER_I2C;
    interface_fd = wiringPiI2CSetup(target_addr);
}

void IOLinker::registerInterrupt(uint8_t intpin, IOLinker::callback_t callback)
{
    pinMode(intpin, INPUT);
    pullUpDnControl(intpin, PUD_UP);
    wiringPiISR(intpin, INT_EDGE_FALLING, callback);
}

#elif defined(ARDUINO)
void IOLinker::registerInterrupt(uint8_t intpin)
{
    pinMode(intpin, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(intpin), callback, FALLING);
   *digitalPinToPCMSK(intpin) |= bit (digitalPinToPCMSKbit(intpin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(intpin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(intpin)); // enable interrupt for the group 
}
#endif


/** Messages **/
        
bool IOLinker::sendBuf(uint8_t *msglist, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        /* Seek to first command */
        if (!cmdBitOn(*msglist)) {
            msglist++;
            continue;
        }

        uint8_t msglength = 0, replyargcount = 0;

        /* Find next command to derive message length */
        for (; msglength < size; msglength++) {
            if (cmdBitOn(msglist[1 + msglength])) {
                break;
            }
        }

        /* We don't execute read messages. They have no place here. */
        if (!rwBitOn(*msglist)) {
            writeMsg(msglist, msglength + 1);
            finishAndReadReply();
        }

        /* Jump to next command */
        msglist += msglength;
        i += msglength;
        size -= msglength + 1;
    }

    return true;
}
        
uint16_t IOLinker::version(void)
{
    uint8_t buf[2 + IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    buf[0] = IOLINKER_CMD_VER;
    writeMsg(buf, 1);
    if (finishAndReadReply(buf, 2 + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT) < 2) {
        return 0;
    }
    return (argByte(buf, sizeof(buf), 0) << 8 | argByte(buf, sizeof(buf), 1));
}

void IOLinker::setPinType(pin_types type, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { IOLINKER_CMD_TYP,
            argData(pin_start), argData(pin_start >> 7),
            argData(pin_end), argData(pin_end >> 7),
            argData(type) };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

uint8_t IOLinker::readRegister(uint8_t addr)
{
    uint8_t tx[] = { IOLINKER_CMD_REA,
            0x27, addr, 0, 0 };
    uint8_t buf[1 + IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeMsg(tx, sizeof(tx));
    if (finishAndReadReply(buf, 1 + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT) < sizeof(buf)) {
        return 0;
    }
    return argByte(buf, sizeof(buf), 0);
}

bool IOLinker::readInput(uint16_t pin)
{
    uint8_t tx[] = { IOLINKER_CMD_REA,
            argData(pin), argData(pin >> 7), 0, 0 };
    uint8_t buf[1 + IOLINKER_REPLY_MAXMETA_BYTECOUNT];
    writeMsg(tx, sizeof(tx));
    if (finishAndReadReply(buf, 1 + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT) < sizeof(buf)) {
        return false;
    }
    return ((argByte(buf, sizeof(buf), 0) >> 6) == 1);
}

void IOLinker::readInput(uint8_t *s, uint8_t len, uint16_t pin_start,
        uint16_t pin_end)
{
    if (pin_end < pin_start) {
        pin_end = 0;
    }

    uint8_t bytecount = (uint8_t)(
            (pin_distance(pin_start, pin_end) + 7.5) / 7);
    if (bytecount < len) {
        len = bytecount;
    }

    uint8_t tx[] = { IOLINKER_CMD_REA,
            argData(pin_start), argData(pin_start >> 7),
            argData(pin_end), argData(pin_end >> 7) };
    writeMsg(tx, sizeof(tx));

    if (finishAndReadReply(s, len + optionalMetaByteCount() +
                IOLINKER_REPLY_META_BYTECOUNT) < len) {
        return;
    }
    
    uint8_t offset = 1, j = 0, lastbyte = 0;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = (argByte(s, sizeof(s), 0 + j) << offset);
        byte |= (argByte(s, sizeof(s), 1 + j) >> (7 - offset));

        s[j] = byte;

        if (++offset >= 8) {
            offset = 1;
        } else {
            j++;
        }
    }
}

void IOLinker::setOutput(bool state, uint16_t pin_start, uint16_t pin_end)
{
    if (pin_end < pin_start) {
        pin_end = 0;
    }

    uint8_t bytecount = 5 + (uint8_t)(
            (pin_distance(pin_start, pin_end) + 7.5) / 7);
    uint8_t buf[bytecount], pos = 0;
    buf[pos++] = IOLINKER_CMD_SET;
    buf[pos++] = argData(pin_start);
    buf[pos++] = argData(pin_start >> 7);
    buf[pos++] = argData(pin_end);
    buf[pos++] = argData(pin_end >> 7);
    
    uint8_t byte = ((state) ? 0x7f : 0x00);
    for (; pos < bytecount; pos++) {
        buf[pos] = byte;
    }
    
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::setOutput(uint8_t *s, uint8_t len, uint16_t pin_start,
        uint16_t pin_end)
{
    if (pin_end < pin_start) {
        pin_end = 0;
    }

    uint8_t bytecount = 5 + max(1,
            (uint8_t)((pin_distance(pin_start, pin_end) + 7.5) / 7));
    uint8_t buf[bytecount], pos = 0;
    buf[pos++] = IOLINKER_CMD_SET;
    buf[pos++] = argData(pin_start);
    buf[pos++] = argData(pin_start >> 7);
    buf[pos++] = argData(pin_end);
    buf[pos++] = argData(pin_end >> 7);
    
    uint8_t offset = 1, j = 0;

    for (; pos < bytecount; pos++) {
        uint8_t byte = (s[j] >> offset);
        
        if (j > 0) {
            byte |= (s[j-1] << (7 - offset));
        }

        buf[pos] = byte;

        if (++offset >= 8) {
            offset = 1;
        } else {
            j++;
        }
    }

    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::beginBuffering(void)
{
    uint8_t buf[] = { IOLINKER_CMD_SYN };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::executeBuffer(void)
{
    uint8_t buf[] = { IOLINKER_CMD_TRG };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::link(uint16_t target_pin, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { IOLINKER_CMD_LNK,
            argData(pin_start), argData(pin_start >> 7),
            argData(pin_end), argData(pin_end >> 7),
            argData(target_pin), argData(target_pin >> 7), };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::pwm(uint8_t pwm_r, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { IOLINKER_CMD_PWM,
            argData(pin_start), argData(pin_start >> 7),
            argData(pin_end), argData(pin_end >> 7),
            argData(pwm_r), };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::pwmPeriod(uint8_t per)
{
    uint8_t buf[] = { IOLINKER_CMD_PER, argData(per) };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::clearPinFunctions(uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { IOLINKER_CMD_CLR,
        argData(pin_start), argData(pin_start >> 7),
        argData(pin_end), argData(pin_end >> 7) };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}

void IOLinker::reset(void)
{
    uint8_t buf[] = { IOLINKER_CMD_RST };
    writeMsg(buf, sizeof(buf));
    finishAndReadReply();
}


/** Chip chain scanning **/

uint16_t IOLinker::firstAddress(void)
{
    for (uint8_t addr = 0; addr <= IOLINKER_TARGET_MAX; addr++) {
        targetAddress(addr);

        if (available()) {
            return addr;
        }
    }

    return 128;
}

uint16_t IOLinker::chainLength(uint8_t start)
{
    uint8_t len = 0;
    uint8_t bak = target_addr;
    
    do {
        targetAddress(start + len);
        len++;
        
        if ((start + len) > IOLINKER_TARGET_MAX) {
            break;
        }
    } while (available());

    targetAddress(bak);
    return len - 1;
}


uint8_t IOLinker::crc7(uint8_t *s, uint8_t len, uint8_t crc)
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

uint8_t IOLinker::finishAndReadReply(uint8_t *s, uint8_t len)
{
    int i = 0;

#if defined(WIRINGPI) || defined(__PC)
    if (interface_mode == IOLINKER_I2C || interface_mode == IOLINKER_SPI ||
            interface_mode == IOLINKER_UART) {
        if (interface_fd == -1) {
            return i;
        }
        if (len == 0 || s == NULL) {
            return i;
        }
       
        for (int timeout = 10; (!serialDataAvail(interface_fd)
                && timeout > 0); timeout--) {
#ifdef __PC
            usleep(10000);
#else
            delayMicroseconds(10000);
#endif
        }

        if (serialDataAvail(interface_fd)) {
            i = read(interface_fd, s, len);
            //printf("Got data of length %d! %x %x\n", i, s[0], s[1]);
            
            /*if (i < 0) {
                i = 0;
            }*/
        }

        // TODO: For SPI, unset SS
    }
#elif defined(ARDUINO)
    if (interface_mode == IOLINKER_I2C) {
        Wire.endTransmission();
        if (len > 0) {
            Wire.requestFrom(target_addr, len);
            for (; i < len && Wire.available(); i++) {
                s[i] = (uint8_t)Wire.read();
            }
        }
    } else if (interface_mode == IOLINKER_SPI) {
        if (len > 0) {
            memset(s, '\0', len);
            SPI.transfer(s, len);
        }
        i = len;
        digitalWrite(__IOLINKER_SPI_CS, HIGH); // unselect
        SPI.endTransaction();
    } else if (interface_mode == IOLINKER_UART) {
        for (; i < len; i++) {
            s[i] = interface_stream->read();
        }
    }
#endif
    else if (interface_mode == IOLINKER_INTERFACE_CALLBACK) {
        i = interface_testfunc(interface_buf_reset,
                interface_buf - interface_buf_reset);

        /* Reply is in '*s', and is of length 'i' */
        if (s != NULL && len > 0) {
            interface_buf = interface_buf_reset;
            strncpy((char *)s, (const char *)interface_buf, i);
        }
    }

    if (len == 0 || s == NULL) {
        return i;
    }

    /* Verify length of return message */
    if (i < len || i < (2 + optionalMetaByteCount())) {
        return i;
    }

    /* Verify CRC */
    if (crcBitOn(cmdByte(s, len))) {
        s[i - 1] <<= 1;
        
        if (crc7(s, i) != 0) {
            return i;
        }
    }

    return i;
}

void IOLinker::writeMsg(uint8_t *s, uint8_t len)
{
    s[0] |= IOLINKER_BITMASK_CMD_BIT;
    newCmd();

#ifdef WIRINGPI
    /* Should work for UART, SPI and I2C alike. */
    if (interface_mode == IOLINKER_I2C ||
            interface_mode == IOLINKER_SPI ||
            interface_mode == IOLINKER_UART) {
        if (interface_fd == -1) {
            //return IOLINKER_ERROR_INTERFACE;
            return;
        }
        
        /* In UART mode, we send out the target address in an extra
           byte manually. */
        if (interface_mode == IOLINKER_UART ||
                interface_mode == IOLINKER_INTERFACE_CALLBACK) {
            uint8_t tmp = target_addr;
            write(interface_fd, &tmp, 1);
        }

        write(interface_fd, s, len);
    }
#elif defined(ARDUINO)
    if (interface_mode == IOLINKER_I2C) {
        Wire.write(s, len);
    } else if (interface_mode == IOLINKER_SPI) {
        SPI.transfer(s, len);
    } else if (interface_mode == IOLINKER_UART) {
        /* In UART mode, we send out the target address in an extra
           byte manually. */
        if (interface_mode == IOLINKER_UART ||
                interface_mode == IOLINKER_INTERFACE_CALLBACK) {
            uint8_t tmp = target_addr;
            interface_stream->write(&tmp, 1);
        }

        interface_stream->write(s, len);
    }
#else
    if (interface_mode == IOLINKER_UART) {
        if (interface_fd == -1) {
            //return IOLINKER_ERROR_INTERFACE;
            return;
        }
        
        /* In UART mode, we send out the target address in an extra
           byte manually. */
        if (interface_mode == IOLINKER_UART ||
                interface_mode == IOLINKER_INTERFACE_CALLBACK) {
            uint8_t tmp = target_addr;
            write(interface_fd, &tmp, 1);
        }
        
        write(interface_fd, s, len);
    }
#endif

    else if (interface_mode == IOLINKER_INTERFACE_CALLBACK) {
        if ((interface_buf + len + 1) <= interface_buf_end) {
            /* We send out the target address in an extra byte manually. */
            interface_buf[0] = target_addr;
            interface_buf++;

            strncpy((char *)interface_buf, (const char *)s, (size_t)len);
            interface_buf += len;
        }
    }

    __crc = crc7(s, len, __crc);
}

void IOLinker::newCmd()
{
    __crc = 0;
#ifdef __PC
    interface_buf = interface_buf_reset;
#endif

#ifdef WIRINGPI
    /* Nothing to do on Raspberry */
    // TODO: For SPI, set SS
#elif defined(ARDUINO)
    if (interface_mode == IOLINKER_SPI) {
        digitalWrite(__IOLINKER_SPI_CS, LOW); // select
        SPI.beginTransaction(__IOLINKER_SPI_SETTINGS); 
    } else if (interface_mode == IOLINKER_I2C) {
        Wire.beginTransmission(target_addr);
    } else if (interface_mode == IOLINKER_UART) {
        /* Nothing to prepare for UART transmission */
    }
#else
    /* Nothing to prepare for PC UART transmission */
#endif
}

