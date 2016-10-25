/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/IOLinker
 **/

/**
 * @file IOLinker_unittest.cpp
 * @author Julian von Mendel
 * @brief Quick test program for IOLinker class
 **/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "IOLinker.h"

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

uint8_t IOLinker_sim(unsigned char *s, uint8_t len)
{
    sim_success = 0;

    /* If we simulate a non-existant node, we quit here */
    if (sim_mode == SIM_MODE_NOTTHERE) {
        sim_success = 1;
        return 0;
    }
    
    /* Verify received message */
    if (sim_mode == SIM_MODE_CHECK &&
            (strncmp((const char *)s, (const char *)buf_expectedmsg,
                std::min(len, expectedmsg_len)) != 0 ||
                expectedmsg_len != len)) {
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

IOLinker IOLinker;

void test_crc(void)
{
    unsigned char 
         msg1[] = { 0xD1, IOLinker::IOLINKER_TARGET_FIRST, 0x05 },
         msg2[] = { 0xD1, IOLinker::IOLINKER_TARGET_FIRST, 0x34, 0x02, 0x3b },
         msg3[] = { 0xD1, IOLinker::IOLINKER_TARGET_FIRST, 0x12, 0x02, 0x3b };

    assert(IOLinker.crc7(msg1, sizeof(msg1) - 1) == msg1[sizeof(msg1) - 1]);
    assert(IOLinker.crc7(msg2, sizeof(msg2) - 1) == msg2[sizeof(msg2) - 1]);
    assert(IOLinker.crc7(msg2 + 1, sizeof(msg2) - 2, IOLinker.crc7(msg2, 1))
            == msg2[sizeof(msg2) - 1]);

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(true);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    IOLinker.version();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
    
    fakereply_len = sizeof(msg3);
    strncpy((char *)buf_fakereply, (const char *)msg3, fakereply_len);

    sim_success = 0;
    IOLinker.version();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_ERROR_CRC);
}

void testmsg_ver(void)
{
    unsigned char msg1[] = { 0xC1, IOLinker::IOLINKER_TARGET_ALL },
         msg2[] = { 0xC1, IOLinker::IOLINKER_TARGET_FIRST },
         msg3[] = { 0xC1, IOLinker::IOLINKER_TARGET_FIRST, 0x41, 0x01 }, /* reply: version 1, pro, 49 pins */
         msg4[] = { 0xD1, IOLinker::IOLINKER_TARGET_MAX, 0x34 },
         msg5[] = { 0xD1, IOLinker::IOLINKER_TARGET_MAX, 0x02, 0x02, 0x43 }, /* reply: version 2, basic, 64 pins */
         msg6[] = { 0x82 },
         msg7[] = { 0x82, 0x7f }; /* reply: error */

    /* Test version message with no reply */
    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_NOTTHERE;

    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_ALL);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);

    sim_success = 0;
    assert(IOLinker.available() == false);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_ERROR_NOREPLY);

    /* Test version message */
    sim_mode = SIM_MODE_CHECK;
    uint16_t version;

    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    //IOLinker.buffer(true);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg2);
    strncpy((char *)buf_expectedmsg, (const char *)msg2, expectedmsg_len);
    fakereply_len = sizeof(msg3);
    strncpy((char *)buf_fakereply, (const char *)msg3, fakereply_len);

    sim_success = 0;
    version = IOLinker.version();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
    assert(IOLinker.pinCount(version) == 49);
    assert(IOLinker.isProVersion(version) == true);
    
    /* Test version different message with different reply */
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_MAX);
    //IOLinker.buffer(false);
    IOLinker.crc(true);
    
    expectedmsg_len = sizeof(msg4);
    strncpy((char *)buf_expectedmsg, (const char *)msg4, expectedmsg_len);
    fakereply_len = sizeof(msg5);
    strncpy((char *)buf_fakereply, (const char *)msg5, fakereply_len);

    sim_success = 0;
    version = IOLinker.version();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
    assert(IOLinker.pinCount(version) == 64);
    assert(IOLinker.isProVersion(version) == false);
}

void testmsg_syn(void)
{
    unsigned char 
         msg1[] = { 0x88, IOLinker::IOLINKER_TARGET_FIRST },
         msg2[] = { 0x88, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.syncOutputsToBuffer();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_trg(void)
{
    unsigned char 
         msg1[] = { 0x89, IOLinker::IOLINKER_TARGET_FIRST },
         msg2[] = { 0x89, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.syncBufferToOutputs();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_rst(void)
{
    unsigned char 
         msg1[] = { 0x8f, IOLinker::IOLINKER_TARGET_FIRST },
         msg2[] = { 0x8f, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.reset();
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_typ(void)
{
    unsigned char 
         msg1[] = { 0x82, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x01, 0x00, 0x03 },
         msg2[] = { 0x82, IOLinker::IOLINKER_TARGET_FIRST };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.setPinType(IOLinker::IOLINKER_OUTPUT, 1, 128);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_clr(void)
{
    unsigned char 
         msg1[] = { 0x8a, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x01, 0x00 };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.clearPinFunctions(1, 128);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_rea(void)
{
    unsigned char 
         msg1[] = { 0xC7, IOLinker::IOLINKER_TARGET_FIRST, 0x27, 0x7f, 0x00, 0x00, },
         msg2[] = { 0xC7, IOLinker::IOLINKER_TARGET_FIRST, 0x00 };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    uint8_t buf;
    assert(!IOLinker.readRegister(0x7f));
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_rea2(void)
{
    unsigned char 
         msg1[] = { 0xC7, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x05, 0x00, 0x00, },
         msg2[] = { 0xC7, IOLinker::IOLINKER_TARGET_FIRST, 0x00 },
         msg3[] = { 0xC7, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x07, },
         msg4[] = { 0xC7, IOLinker::IOLINKER_TARGET_FIRST, 0x55 };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);

    sim_success = 0;
    uint8_t buf;
    assert(!IOLinker.readInput(5));
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);

    expectedmsg_len = sizeof(msg3);
    strncpy((char *)buf_expectedmsg, (const char *)msg3, expectedmsg_len);
    fakereply_len = sizeof(msg4);
    strncpy((char *)buf_fakereply, (const char *)msg4, fakereply_len);
    
    IOLinker.readInput(&buf, sizeof(buf), 1, 7);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
    assert(buf == (0x55 << 1));
}

void testmsg_set(void)
{
    unsigned char 
         msg1[] = { 0x83, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x40 },
         msg2[] = { 0x83, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS },
         msg3[] = { 0x83, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x7f };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    uint8_t buf = 0x40 << 1;
    IOLinker.setOutput(&buf, sizeof(buf), 1, 0);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);

    expectedmsg_len = sizeof(msg3);
    strncpy((char *)buf_expectedmsg, (const char *)msg3, expectedmsg_len);

    sim_success = 0;
    IOLinker.setOutput(true, 1, 0);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_lnk(void)
{
    unsigned char 
         msg1[] = { 0x84, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02 },
         msg2[] = { 0x84, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.link(2, 1, 0);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_pwm(void)
{
    unsigned char 
         msg1[] = { 0x85, IOLinker::IOLINKER_TARGET_FIRST, 0x00, 0x01, 0x00, 0x02, 0x3f },
         msg2[] = { 0x85, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.pwm(0x3f, 1, 2);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

void testmsg_per(void)
{
    unsigned char 
         msg1[] = { 0x86, IOLinker::IOLINKER_TARGET_FIRST, 0x7f },
         msg2[] = { 0x86, IOLinker::IOLINKER_TARGET_FIRST, IOLinker::IOLINKER_STATUS_SUCCESS };

    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
    IOLinker.targetAddress(IOLinker::IOLINKER_TARGET_FIRST);
    IOLinker.buffer(false);
    IOLinker.crc(false);

    expectedmsg_len = sizeof(msg1);
    strncpy((char *)buf_expectedmsg, (const char *)msg1, expectedmsg_len);
    fakereply_len = sizeof(msg2);
    strncpy((char *)buf_fakereply, (const char *)msg2, fakereply_len);
    fakereply_len = 0;

    sim_success = 0;
    IOLinker.pwmPeriod(0x7f);
    assert(sim_success == 1);
    //assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_SUCCESS);
}

uint8_t IOLinker_sim_range(unsigned char *s, uint8_t len)
{
    if (len != 2) {
        return 0;
    }

    unsigned char expect[] = { 0xC1 },
         reply[] = { 0xC1, s[1], 0x41, 0x01 },

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
    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim_range,
            (uint8_t *)buf0, sizeof(buf0));
    sim_mode = SIM_MODE_CHECK;
   
    assert(IOLinker.firstAddress() == 30);
    assert(IOLinker.chainLength(0) == 0);
    assert(IOLinker.chainLength(32) == 16);
    assert(IOLinker.chainLength(64) == 0);
}


int main(void)
{
    unsigned char buf[] = { 0xf5, 0x0a, 0x00, 0x01, 0x02, 0x0c };
    uint8_t msg_ver = {  };
    uint8_t msg_ver_reply = {  };
    
    /* Test bit & byte methods */
    assert(IOLinker.cmdByte(buf, sizeof(buf)) == buf[0]);
    assert(IOLinker.addrByte(buf, sizeof(buf)) == buf[1]);
    for (int i = 0; i < 3; i++) {
        assert(IOLinker.argByte(buf, sizeof(buf), i) == buf[1 + i]);
    }
    assert(IOLinker.crcByte(buf, sizeof(buf)) == buf[sizeof(buf) - 1]);

    assert(IOLinker.cmdBitOn(IOLinker::IOLINKER_BITMASK_CMD_BIT) == true);
    assert(IOLinker.cmdBitOn(0) == false);
    assert(IOLinker.rwBitOn(IOLinker::IOLINKER_BITMASK_RW_BIT) == true);
    assert(IOLinker.rwBitOn(0) == false);
    assert(IOLinker.bufBitOn(IOLinker::IOLINKER_BITMASK_BUF_BIT) == true);
    assert(IOLinker.bufBitOn(0) == false);
    assert(IOLinker.crcBitOn(IOLinker::IOLINKER_BITMASK_CRC_BIT) == true);
    assert(IOLinker.crcBitOn(0) == false);
    assert(IOLinker.commandCode(IOLinker::IOLINKER_CMD_VER) == IOLinker::IOLINKER_CMD_VER);
    assert(IOLinker.commandCode(IOLinker::IOLINKER_CMD_TYP) == IOLinker::IOLINKER_CMD_TYP);
    assert(IOLinker.argData(0xff) == 0x7f);
  
    /* Test behaviour of uninitialized class (error case) */
    assert(IOLinker.statusCode() == IOLinker::IOLINKER_STATUS_UNDEFINED);
    assert(IOLinker.chainLength() == 0);
    assert(IOLinker.available() == false);

    /* Initialize class */
    IOLinker.beginTest((IOLinker::testfunc_t)IOLinker_sim,
            (uint8_t *)buf0, sizeof(buf0));
    assert(IOLinker.addrByteCount() == 1);

    /* Test messages */
    test_crc();
    testmsg_ver();
    testmsg_syn();
    testmsg_trg();
    testmsg_rst();
    testmsg_typ();
    testmsg_clr();
    testmsg_rea();
    testmsg_rea2();
    testmsg_set();
    testmsg_lnk();
    testmsg_pwm();
    testmsg_per();
    testmsg_chainlength_firstaddress();

    fprintf(stderr, "%d tests failed.\n", failed);
}

