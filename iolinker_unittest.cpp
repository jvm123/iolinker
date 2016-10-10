/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/iolinker
 * Quick test program for iolinker class
 **/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "iolinker.h"

uint8_t failed = 0;
#define assert(pass) __assert(pass, __LINE__)
void __assert(bool pass, unsigned int line)
{
    if (pass) {
        return;
    }
    failed++;
    fprintf(stderr, "Unit test failure on line %d.\n", line);
}

uint8_t buf0[255];
enum sim_mode {
    SIM_MODE_NOTTHERE = 0,
    SIM_MODE_DAMAGED,
    SIM_MODE_BASIC,
    SIM_MODE_PRO,
} sim_mode = SIM_MODE_NOTTHERE;

uint8_t iolinker_sim(unsigned char *s, uint8_t len)
{
    return 0;
}

iolinker iolinker;

int main(void)
{
    unsigned char buf[] = { 0xf5, 0x0a, 0x00, 0x01, 0x02, 0x0c };
    
    /* Initialize class */
    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim, buf0);
    assert(iolinker.addrByteCount() == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_UNDEFINED);
    assert(iolinker.available() == false);
    assert(iolinker.statusCode() == iolinker::ERROR_NOREPLY);

    assert(iolinker.cmdByte(buf, sizeof(buf)) == buf[0]);
    assert(iolinker.addrByte(buf, sizeof(buf)) == buf[1]);
    for (int i = 0; i < 3; i++) {
        assert(iolinker.argByte(buf, sizeof(buf), i) == buf[2 + i]);
    }
    assert(iolinker.crcByte(buf, sizeof(buf)) == buf[sizeof(buf) - 1]);

    assert(iolinker.cmdBitOn(iolinker::BITMASK_CMD_BIT) == true);
    assert(iolinker.cmdBitOn(0) == false);
    assert(iolinker.rwBitOn(iolinker::BITMASK_RW_BIT) == true);
    assert(iolinker.rwBitOn(0) == false);
    assert(iolinker.bufBitOn(iolinker::BITMASK_BUF_BIT) == true);
    assert(iolinker.bufBitOn(0) == false);
    assert(iolinker.crcBitOn(iolinker::BITMASK_CRC_BIT) == true);
    assert(iolinker.crcBitOn(0) == false);
    assert(iolinker.commandCode(0x55) == 0x05);
    assert(iolinker.argData(0xff) == 0x7f);

    assert(iolinker.send(buf, sizeof(buf) == false));

    iolinker.targetAddress(iolinker::TARGET_ALL);
    iolinker.buffer(false);
    iolinker.crc(false);


    assert(iolinker.chainLength() == 0);
    assert(iolinker.firstAddress() == 0);
    assert(iolinker.send(0, 0) == false);
}

