/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/iolinker
 * iolinker class for Arduino, Raspberry Pi and PC unit tests
 **/

#include "iolinker.h"

#ifdef WIRINGPI
void iolinker::beginSerial(unsigned char *dev)
{
    interface_mode = UART;
    interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE);
}

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
void iolinker::beginStream(Stream &stream)
{
    interface_mode = UART;
    interface_stream = &stream;
}

void iolinker::beginSPI(void)
{
    interface_mode = SPI;
    SPI.begin();
    initSS();
    SPI.setClockDivider(SPI_CS, 21);
    SPI.setDataMode(SPI_CS, SPI_MODE0);
}

void iolinker::beginI2C(void)
{
    interface_mode = I2C;
    Wire.begin();
}
#else
void iolinker::beginSerial(unsigned char *dev)
{
    interface_mode = UART;
    // TODO: PC solution
    interface_fd = serialOpen(dev, __IOLINKER_BAUDRATE);
}
#endif

void iolinker::beginTest(iolinker::testfunc_t testfunc, uint8_t *buf)
{
    interface_mode = INTERFACE_UNCLEAR;
    interface_testfunc = testfunc;
    interface_buf = buf;
    interface_buf_reset = buf;
}


/** Message preparation **/

void iolinker::targetAddress(uint8_t addr)
{
#ifdef WIRINGPI
    if (interface_mode == I2C && target_addr != addr) {
        interface_fd = wiringPiI2CSetup(addr);
    }
#endif

    target_addr = addr;
}

void iolinker::buffer(bool active)
{
    if (active) {
        cmdbyte |= BITMASK_BUF_BIT;
    } else {
        cmdbyte &= ~(BITMASK_BUF_BIT);
    }
}

void iolinker::crc(bool active)
{
    if (active) {
        cmdbyte |= BITMASK_CRC_BIT;
    } else {
        cmdbyte &= ~(BITMASK_CRC_BIT);
    }
}

/** Status codes **/

iolinker::status_code iolinker::statusCode(void)
{
    return status;
}

bool iolinker::available(void)
{
    version();
    return (statusCode() == STATUS_SUCCESS);
}


/** Messages **/

uint8_t iolinker::version(void)
{
    uint8_t buf;
    writeCmd(CMD_VER);
    writeCRC();
    readReply(&buf, 0);
    return buf;
}

uint16_t iolinker::pinCount(uint8_t version)
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
        default:
    }
    return 0;
}

bool iolinker::isProVersion(uint8_t _version)
{
    return ((version & (1 << 7)) == 1);
}

void iolinker::setPinType(pin_types type, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7). argData(pin_end),
            argData(type) };
    writeCmd(CMD_TYP);
    write(buf, sizeof(buf));
    writeCRC();
    readReply();
}

bool iolinker::readInput(uint16_t pin)
{

}

void iolinker::readInput(uint8_t *s, uint8_t len, uint16_t pin_start, uint16_t pin_end)
{

}

void iolinker::setOutput(bool state, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t bytecount = max(1,
            (uint8_t)((pin_distance(pin_start, pin_end) + 0.5) / 7));
    uint8_t buf[4] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(CMD_SET);
    write(buf, sizeof(buf));
    
    uint8_t byte = ((state) ? 0xff : 0x00);
    for (uint8_t i = 0; i < bytecount; i++) {
        write(&byte, 1);
    }
    
    writeCRC();
    readReply();
}

void iolinker::setOutput(uint8_t *s, uint8_t len, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t bytecount = max(1,
            (uint8_t)((pin_distance(pin_start, pin_end) + 0.5) / 7));
    uint8_t buf[4] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7), argData(pin_end) };
    writeCmd(CMD_SET);
    write(buf, sizeof(buf));
    
    for (uint8_t i = 0; i < len; i++) {
        // TODO
        uint8_t byte = s[i] >> (i + 1);
        left = s[i] & (0xff >> (8 - i - 1));
        i++;
        byte = (left << 6) | (s[i] >> (i + 1));
        left = s[i] & (0xff >> (8 - i - 1));
        i++;
        byte = (left << 5) | (s[i] >> (i + 1));
        left = s[i] & (0xff >> (8 - i - 1));

        write(&byte, 1);
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
            argData(pin_end >> 7). argData(pin_end),
            argData(target_pin >> 7), argData(target_pin), };
    writeCmd(CMD_PWM);
    write(buf, sizeof(buf));
    writeCRC();
    readReply();
}

void iolinker::pwm(uint8_t pwm_r, uint16_t pin_start, uint16_t pin_end)
{
    uint8_t buf[] = { argData(pin_start >> 7), argData(pin_start),
            argData(pin_end >> 7). argData(pin_end),
            argData(pwm_r), };
    writeCmd(CMD_PWM);
    write(buf, sizeof(buf));
    writeCRC();
    readReply();
}

void iolinker::pwmPeriod(uint8_t per)
{
    per = argData(per);
    writeCmd(CMD_PER);
    write(&per, 1);
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
    // TODO
}

uint16_t iolinker::chainLength(uint8_t start)
{
    // TODO
}

bool iolinker::readReply(uint8_t *s, uint8_t len)
{
#ifdef WIRINGPI
        // TODO
#elif ARDUINO
    if (interface_mode == I2C) {
        // TODO
        Wire.endTransmission();
    } else if (interface_mode == SPI) {
        resetSS();
        // TODO
    } else if (interface_mode == UART) {
        for (uint8_t i = 0; i < len; i++) {
            s[i] = interface_stream->read();
        }
    }
#elif __PC
    interface_buf = interface_buf_reset;
    size = interface_testfunc(s, len);
    /* Reply is in '*s', and is of length 'size' */
#endif

    // TODO
    status = ERROR_NOREPLY;
    return false;
}

void iolinker::write(uint8_t *s, uint8_t len)
{
    // TODO
#ifdef WIRINGPI
    /* Should work for UART, SPI and I2C alike. */
    write(interface_fd, s, len);
#elif ARDUINO
    for (uint8_t i = 0; i < len; i++) {
        if (interface_mode == I2C) {
            Wire.write(s[i]);
        } else if (interface_mode == SPI) {
            SPI.transfer(s[i]);
        } else if (interface_mode == UART) {
            interface_stream->write(s[i]);
        }
    }
#else
    if (interface_mode == UART) {
        write(interface_fd, s, len);
    } else if (interface_mode == INTERFACE_UNCLEAR) {
        strncpy(interface_buf, s, len);
        interface_buf += len;
    }
#endif
}

void iolinker::writeCRC(void)
{
    write(&crc, 1);
}

void iolinker::writeCmd(uint8_t *s, uint8_t len, uint8_t replylen)
{
    crc = 0;
#ifdef __PC
    interface_buf = interface_buf_reset;
#endif

    cmdbyte &= ~(BITMASK_CMD);
    cmdbyte |= cmd;

#ifdef WIRINGPI
    // Nothing to do on Raspberry
#elif ARDUINO
    if (interface_mode == SPI) {
        setSS();
    } else if (interface_mode == I2C) {
        Wire.beginTransmission(target_addr);
    } else if (interface_mode == UART) {
        // Nothing to do for UART
    }
#else
    // Nothing to do on PC
#endif

    write(&cmdbyte, 1);
}

