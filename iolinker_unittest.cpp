/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/iolinker
 **/

/**
 * @file iolinker_unittest.cpp
 * @author Julian von Mendel
 * @brief Quick test program for iolinker class
 **/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
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

unsigned char buf0[255], buf_expectedmsg[255], buf_fakereply[255];
uint8_t expectedmsg_len = 0,
        fakereply_len = 0, sim_success = 0;
enum sim_mode {
    SIM_MODE_NOTTHERE = 0,
    SIM_MODE_DAMAGED,
    SIM_MODE_CHECK,
    SIM_MODE_NOCHECK,
} sim_mode = SIM_MODE_NOTTHERE;

uint8_t iolinker_sim(unsigned char *s, uint8_t len)
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

iolinker iolinker;

void test_crc(void)
{
    unsigned char 
         msg1[] = { 0xD1, iolinker::TARGET_FIRST, 0x05 },
         msg2[] = { 0xD1, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS, 0x34, 0x02, 0x6e },
         msg3[] = { 0xD1, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS, 0x34, 0x02, 0x00 };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(true);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    assert(sim_success == 1);
    iolinker.reset();
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
    
    fakereply_len = sizeof(msg3);
    strncpy((char *)buf_fakereply, (const char *)msg3, fakereply_len);

    sim_success = 0;
    iolinker.reset();
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::ERROR_CRC);
}

void testmsg_ver(void)
{
    unsigned char msg1[] = { 0xC1, iolinker::TARGET_ALL },
         msg2[] = { 0xC1, iolinker::TARGET_FIRST },
         msg3[] = { 0xC1, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS, 0x41, 0x01 }, /* reply: version 1, pro, 49 pins */
         msg4[] = { 0xD1, iolinker::TARGET_MAX, 0x34 },
         msg5[] = { 0xD1, iolinker::TARGET_MAX, iolinker::STATUS_SUCCESS, 0x34, 0x02, 0x65 }, /* reply: version 2, basic, 64 pins */
         msg6[] = { 0x82 },
         msg7[] = { 0x82, 0x7f, iolinker::ERROR_ARGCOUNT }; /* reply: error */

    /* Test version message with no reply */
    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_NOTTHERE;

    iolinker.targetAddress(iolinker::TARGET_ALL);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);

    sim_success = 0;
    assert(iolinker.available() == false);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::ERROR_NOREPLY);

    /* Test version message */
    sim_mode = SIM_MODE_CHECK;
    uint16_t version;

    iolinker.targetAddress(iolinker::TARGET_FIRST);
    //iolinker.buffer(true);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg2);
    strncpy((char *)buf_expectedmsg, (const char *)msg2, expectedmsg_len);
    fakereply_len = sizeof(msg3);
    strncpy((char *)buf_fakereply, (const char *)msg3, fakereply_len);

    sim_success = 0;
    version = iolinker.version();
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
    assert(iolinker.pinCount(version) == 49);
    assert(iolinker.isProVersion(version) == true);
    
    /* Test version different message with different reply */
    iolinker.targetAddress(iolinker::TARGET_MAX);
    //iolinker.buffer(false);
    iolinker.crc(true);
    
    expectedmsg_len = sizeof(msg4);
    strncpy((char *)buf_expectedmsg, (const char *)msg4, expectedmsg_len);
    fakereply_len = sizeof(msg5);
    strncpy((char *)buf_fakereply, (const char *)msg5, fakereply_len);

    sim_success = 0;
    version = iolinker.version();
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
    assert(iolinker.pinCount(version) == 64);
    assert(iolinker.isProVersion(version) == false);
}

void testmsg_syn(void)
{
    unsigned char 
         msg1[] = { 0x88, iolinker::TARGET_FIRST },
         msg2[] = { 0x88, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.syncOutputsToBuffer();
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_trg(void)
{
    unsigned char 
         msg1[] = { 0x89, iolinker::TARGET_FIRST },
         msg2[] = { 0x89, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.syncBufferToOutputs();
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_rst(void)
{
    unsigned char 
         msg1[] = { 0x8f, iolinker::TARGET_FIRST },
         msg2[] = { 0x8f, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.reset();
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_typ(void)
{
    unsigned char 
         msg1[] = { 0x82, iolinker::TARGET_FIRST, 0x00, 0x01, 0x01, 0x00, 0x03 },
         msg2[] = { 0x82, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.setPinType(iolinker::OUTPUT, 1, 128);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_rea(void)
{
    unsigned char 
         msg1[] = { 0xC7, iolinker::TARGET_FIRST, 0x00, 0x05, 0x00, 0x00, },
         msg2[] = { 0xC7, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS, 0x00 },
         msg3[] = { 0xC7, iolinker::TARGET_FIRST, 0x00, 0x01, 0x00, 0x07, },
         msg4[] = { 0xC7, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS, 0x55 };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    uint8_t buf;
    assert(!iolinker.readInput(5));
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);

    expectedmsg_len = sizeof(msg3);
    strncpy((char *)buf_expectedmsg, (const char *)msg3, expectedmsg_len);
    fakereply_len = sizeof(msg4);
    strncpy((char *)buf_fakereply, (const char *)msg4, fakereply_len);
    
    iolinker.readInput(&buf, sizeof(buf), 1, 7);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
    assert(buf == (0x55 << 1));
}

void testmsg_set(void)
{
    unsigned char 
         msg1[] = { 0x83, iolinker::TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x40 },
         msg2[] = { 0x83, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS },
         msg3[] = { 0x83, iolinker::TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x7f };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    uint8_t buf = 0x40 << 1;
    iolinker.setOutput(&buf, sizeof(buf), 1, 0);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);

    expectedmsg_len = sizeof(msg3);
    strncpy((char *)buf_expectedmsg, (const char *)msg3, expectedmsg_len);

    sim_success = 0;
    iolinker.setOutput(true, 1, 0);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_lnk(void)
{
    unsigned char 
         msg1[] = { 0x84, iolinker::TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02 },
         msg2[] = { 0x84, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.link(2, 1, 0);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_pwm(void)
{
    unsigned char 
         msg1[] = { 0x85, iolinker::TARGET_FIRST, 0x00, 0x01, 0x00, 0x02, 0x3f },
         msg2[] = { 0x85, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.pwm(0x3f, 1, 2);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

void testmsg_per(void)
{
    unsigned char 
         msg1[] = { 0x86, iolinker::TARGET_FIRST, 0x7f },
         msg2[] = { 0x86, iolinker::TARGET_FIRST, iolinker::STATUS_SUCCESS };

    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    iolinker.targetAddress(iolinker::TARGET_FIRST);
    iolinker.buffer(false);
    iolinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    iolinker.pwmPeriod(0x7f);
    assert(sim_success == 1);
    assert(iolinker.statusCode() == iolinker::STATUS_SUCCESS);
}

uint8_t iolinker_sim_range(unsigned char *s, uint8_t len)
{
    if (len != 2) {
        return 0;
    }

    unsigned char expect[] = { 0xC1 },
         reply[] = { 0xC1, s[1], iolinker::STATUS_SUCCESS, 0x41, 0x01 },

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
    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim_range,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    
    assert(iolinker.firstAddress() == 30);
    assert(iolinker.chainLength(0) == 0);
    assert(iolinker.chainLength(32) == 16);
    assert(iolinker.chainLength(64) == 0);
}


int main(void)
{
    unsigned char buf[] = { 0xf5, 0x0a, 0x00, 0x01, 0x02, 0x0c };
    uint8_t msg_ver = {  };
    uint8_t msg_ver_reply = {  };
    
    /* Test bit & byte methods */
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
    assert(iolinker.commandCode(iolinker::CMD_VER) == iolinker::CMD_VER);
    assert(iolinker.commandCode(iolinker::CMD_TYP) == iolinker::CMD_TYP);
    assert(iolinker.argData(0xff) == 0x7f);
  
    /* Test behaviour of uninitialized class (error case) */
    assert(iolinker.statusCode() == iolinker::STATUS_UNDEFINED);
    assert(iolinker.chainLength() == 0);
    assert(iolinker.available() == false);

    /* Initialize class */
    iolinker.beginTest((iolinker::testfunc_t)iolinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    assert(iolinker.addrByteCount() == 1);

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

