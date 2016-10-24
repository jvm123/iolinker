/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/IOlinker
 **/

/**
 * @file IOlinker_unittest.cpp
 * @author Julian von Mendel
 * @brief Quick test program for IOlinker class
 **/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "IOlinker.h"

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

unsigned char buf0[255], buf_expectedmsg[255], buf_fakereply[255];
uint8_t expectedmsg_len = 0,
        fakereply_len = 0, sim_success = 0;
enum sim_mode {
    SIM_MODE_NOTTHERE = 0,
    SIM_MODE_DAMAGED,
    SIM_MODE_CHECK,
    SIM_MODE_NOCHECK,
} sim_mode = SIM_MODE_NOTTHERE;

uint8_t IOlinker_sim(unsigned char *s, uint8_t len)
{
    sim_success = 0;

    /* If we simulate a non-existant node, we quit here */
    if (sim_mode == SIM_MODE_NOTTHERE) {
        sim_success = 1;
        return 0;
    }
    
    /* Verify received message */
    if (sim_mode == SIM_MODE_CHECK &&
            strncmp((const char *)s, (const char *)buf_expectedmsg,
                std::min(len, expectedmsg_len)) != 0) {
        return 0;
    }

    /* Write reply back */
    //len = std::min(fakereply_len, len);
    len = fakereply_len;
    strncpy((char *)s, (const char *)buf_fakereply, len);

    /* Set success flag */
    sim_success = 1;

    /* If we simulate a damaged node, we send back an incomplete reply */
    if (sim_mode == SIM_MODE_DAMAGED) {
        /* Mess up the last byte and don't even return it */
        s[len - 1]++;
        return len - 1;
    }

    return len;
}

IOlinker IOlinker;

void test_crc(void)
{
    unsigned char 
         msg1[] = { 0xD1, IOlinker::IOLINKER_TARGET_FIRST, 0x05 },
         msg2[] = { 0xD1, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS, 0x34, 0x02, 0x6e },
         msg3[] = { 0xD1, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS, 0x12, 0x02, 0x6e };

    assert(IOlinker.crc7(msg1, sizeof(msg1) - 1) == msg1[sizeof(msg1) - 1]);
    assert(IOlinker.crc7(msg2, sizeof(msg2) - 1) == msg2[sizeof(msg2) - 1]);
    assert(IOlinker.crc7(msg2 + 1, sizeof(msg2) - 2, IOlinker.crc7(msg2, 1))
            == msg2[sizeof(msg2) - 1]);

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(true);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.version();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
    
    fakereply_len = sizeof(msg3);
    strncpy((char *)buf_fakereply, (const char *)msg3, fakereply_len);

    sim_success = 0;
    IOlinker.version();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_ERROR_CRC);
}

void testmsg_ver(void)
{
    unsigned char msg1[] = { 0xC1, IOlinker::IOLINKER_TARGET_ALL },
         msg2[] = { 0xC1, IOlinker::IOLINKER_TARGET_FIRST },
         msg3[] = { 0xC1, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS, 0x41, 0x01 }, /* reply: version 1, pro, 49 pins */
         msg4[] = { 0xD1, IOlinker::IOLINKER_TARGET_MAX, 0x34 },
         msg5[] = { 0xD1, IOlinker::IOLINKER_TARGET_MAX, IOlinker::IOLINKER_STATUS_SUCCESS, 0x34, 0x02, 0x65 }, /* reply: version 2, basic, 64 pins */
         msg6[] = { 0x82 },
         msg7[] = { 0x82, 0x7f, IOlinker::IOLINKER_ERROR_ARGCOUNT }; /* reply: error */

    /* Test version message with no reply */
    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_NOTTHERE;

    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_ALL);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);

    sim_success = 0;
    assert(IOlinker.available() == false);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_ERROR_NOREPLY);

    /* Test version message */
    sim_mode = SIM_MODE_CHECK;
    uint16_t version;

    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    //IOlinker.buffer(true);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg2);
    strncpy((char *)buf_expectedmsg, (const char *)msg2, expectedmsg_len);
    fakereply_len = sizeof(msg3);
    strncpy((char *)buf_fakereply, (const char *)msg3, fakereply_len);

    sim_success = 0;
    version = IOlinker.version();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
    assert(IOlinker.pinCount(version) == 49);
    assert(IOlinker.isProVersion(version) == true);
    
    /* Test version different message with different reply */
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_MAX);
    //IOlinker.buffer(false);
    IOlinker.crc(true);
    
    expectedmsg_len = sizeof(msg4);
    strncpy((char *)buf_expectedmsg, (const char *)msg4, expectedmsg_len);
    fakereply_len = sizeof(msg5);
    strncpy((char *)buf_fakereply, (const char *)msg5, fakereply_len);

    sim_success = 0;
    version = IOlinker.version();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
    assert(IOlinker.pinCount(version) == 64);
    assert(IOlinker.isProVersion(version) == false);
}

void testmsg_syn(void)
{
    unsigned char 
         msg1[] = { 0x88, IOlinker::IOLINKER_TARGET_FIRST },
         msg2[] = { 0x88, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.syncOutputsToBuffer();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_trg(void)
{
    unsigned char 
         msg1[] = { 0x89, IOlinker::IOLINKER_TARGET_FIRST },
         msg2[] = { 0x89, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.syncBufferToOutputs();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_rst(void)
{
    unsigned char 
         msg1[] = { 0x8f, IOlinker::IOLINKER_TARGET_FIRST },
         msg2[] = { 0x8f, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.reset();
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_typ(void)
{
    unsigned char 
         msg1[] = { 0x82, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x01, 0x00, 0x03 },
         msg2[] = { 0x82, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.setPinType(IOlinker::IOLINKER_OUTPUT, 1, 128);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_rea(void)
{
    unsigned char 
         msg1[] = { 0xC7, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x05, 0x00, 0x00, },
         msg2[] = { 0xC7, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS, 0x00 },
         msg3[] = { 0xC7, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x07, },
         msg4[] = { 0xC7, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS, 0x55 };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    uint8_t buf;
    assert(!IOlinker.readInput(5));
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);

    expectedmsg_len = sizeof(msg3);
    strncpy((char *)buf_expectedmsg, (const char *)msg3, expectedmsg_len);
    fakereply_len = sizeof(msg4);
    strncpy((char *)buf_fakereply, (const char *)msg4, fakereply_len);
    
    IOlinker.readInput(&buf, sizeof(buf), 1, 7);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
    assert(buf == (0x55 << 1));
}

void testmsg_set(void)
{
    unsigned char 
         msg1[] = { 0x83, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x40 },
         msg2[] = { 0x83, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS },
         msg3[] = { 0x83, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x7f };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    uint8_t buf = 0x40 << 1;
    IOlinker.setOutput(&buf, sizeof(buf), 1, 0);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);

    expectedmsg_len = sizeof(msg3);
    strncpy((char *)buf_expectedmsg, (const char *)msg3, expectedmsg_len);

    sim_success = 0;
    IOlinker.setOutput(true, 1, 0);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_lnk(void)
{
    unsigned char 
         msg1[] = { 0x84, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02 },
         msg2[] = { 0x84, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.link(2, 1, 0);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_pwm(void)
{
    unsigned char 
         msg1[] = { 0x85, IOlinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x02, 0x3f },
         msg2[] = { 0x85, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.pwm(0x3f, 1, 2);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_per(void)
{
    unsigned char 
         msg1[] = { 0x86, IOlinker::IOLINKER_TARGET_FIRST, 0x7f },
         msg2[] = { 0x86, IOlinker::IOLINKER_TARGET_FIRST, IOlinker::IOLINKER_STATUS_SUCCESS };

    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOlinker.targetAddress(IOlinker::IOLINKER_TARGET_FIRST);
    IOlinker.buffer(false);
    IOlinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOlinker.pwmPeriod(0x7f);
    assert(sim_success == 1);
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_SUCCESS);
}

uint8_t IOlinker_sim_range(unsigned char *s, uint8_t len)
{
    if (len != 2) {
        return 0;
    }

    unsigned char expect[] = { 0xC1 },
         reply[] = { 0xC1, s[1], IOlinker::IOLINKER_STATUS_SUCCESS, 0x41, 0x01 },

    sim_success = 0;

    /* If we simulate a non-existant node, we quit here */
    if (sim_mode == SIM_MODE_NOTTHERE) {
        sim_success = 2;
        return 0;
    }
    
    /* Verify received message */
    if (sim_mode == SIM_MODE_CHECK &&
            strncmp((const char *)s, (const char *)expect,
                std::min(len, (uint8_t)sizeof(expect))) != 0) {
        return 0;
    }

    /* Verify range, we only reply for IDs 30 to 47 */
    if (sim_mode == SIM_MODE_CHECK && s[1] < 30 || s[1] > 47) {
        sim_success = 2;
        return 0;
    }

    /* Write reply back */
    len = sizeof(reply);
    strncpy((char *)s, (const char *)reply, len);

    /* Set success flag */
    sim_success = 2;

    /* If we simulate a damaged node, we send back an incomplete reply */
    if (sim_mode == SIM_MODE_DAMAGED) {
        /* Mess up the last byte and don't even return it */
        s[len - 1]++;
        return len - 1;
    }

    return len;
}

void testmsg_chainlength_firstaddress(void)
{
    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim_range,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    
    assert(IOlinker.firstAddress() == 30);
    assert(IOlinker.chainLength(0) == 0);
    assert(IOlinker.chainLength(32) == 16);
    assert(IOlinker.chainLength(64) == 0);
}


int main(void)
{
    unsigned char buf[] = { 0xf5, 0x0a, 0x00, 0x01, 0x02, 0x0c };
    uint8_t msg_ver = {  };
    uint8_t msg_ver_reply = {  };
    
    /* Test bit & byte methods */
    assert(IOlinker.cmdByte(buf, sizeof(buf)) == buf[0]);
    assert(IOlinker.addrByte(buf, sizeof(buf)) == buf[1]);
    for (int i = 0; i < 3; i++) {
        assert(IOlinker.argByte(buf, sizeof(buf), i) == buf[2 + i]);
    }
    assert(IOlinker.crcByte(buf, sizeof(buf)) == buf[sizeof(buf) - 1]);

    assert(IOlinker.cmdBitOn(IOlinker::IOLINKER_BITMASK_CMD_BIT) == true);
    assert(IOlinker.cmdBitOn(0) == false);
    assert(IOlinker.rwBitOn(IOlinker::IOLINKER_BITMASK_RW_BIT) == true);
    assert(IOlinker.rwBitOn(0) == false);
    assert(IOlinker.bufBitOn(IOlinker::IOLINKER_BITMASK_BUF_BIT) == true);
    assert(IOlinker.bufBitOn(0) == false);
    assert(IOlinker.crcBitOn(IOlinker::IOLINKER_BITMASK_CRC_BIT) == true);
    assert(IOlinker.crcBitOn(0) == false);
    assert(IOlinker.commandCode(IOlinker::IOLINKER_CMD_VER) == IOlinker::IOLINKER_CMD_VER);
    assert(IOlinker.commandCode(IOlinker::IOLINKER_CMD_TYP) == IOlinker::IOLINKER_CMD_TYP);
    assert(IOlinker.argData(0xff) == 0x7f);
  
    /* Test behaviour of uninitialized class (error case) */
    assert(IOlinker.statusCode() == IOlinker::IOLINKER_STATUS_UNDEFINED);
    assert(IOlinker.chainLength() == 0);
    assert(IOlinker.available() == false);

    /* Initialize class */
    IOlinker.beginTest((IOlinker::testfunc_t)IOlinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    assert(IOlinker.addrByteCount() == 1);

    /* Test messages */
    test_crc();
    testmsg_ver();
    testmsg_syn();
    testmsg_trg();
    testmsg_rst();
    testmsg_typ();
    testmsg_rea();
    testmsg_set();
    testmsg_lnk();
    testmsg_pwm();
    testmsg_per();
    testmsg_chainlength_firstaddress();

    fprintf(stderr, "%d tests failed.\n", failed);
}

