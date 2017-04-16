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
#include <unistd.h>
#include <algorithm>
#include "IOLinker.h"
#include <wiringPi.h>

/* Initialize class */
IOLinker iolinker;

void resetschematic() {
  iolinker.pwm(127, 0, 48);
  iolinker.clearPinFunctions(0, 48);
  iolinker.setPinType(IOLinker::IOLINKER_INPUT, 0, 48);
}

int main(void)
{
    wiringPiSetupSys();

    //iolinker.beginSPI(0);
    //iolinker.beginI2C();
    iolinker.beginSerial("/dev/ttyAMA0");
    iolinker.targetAddress(0x7f);
    iolinker.setPinType(IOLinker::IOLINKER_INPUT, 32); /* Arduino0 servo output */
    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 47); /* Servo output */
    iolinker.link(32, 47);
return 1;
    if (iolinker.version() != 0x131) {
        printf("ERROR: No response from iolinker (version %x).\n",
                iolinker.version());
        return 0;
    }
    printf("iolinker is responding.\n");

    //resetschematic();

    iolinker.setPinType(IOLinker::IOLINKER_INPUT, 43, 49); /* Arduino 1 LCD connections */
    usleep(10000);
    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 13, 19); /* LCD outputs */
    usleep(10000);

    for (int i = 0; i < 6; i++) {
        //iolinker.link(33 + i, 13 + i); /* Link Arduino 0 to LCD */
        usleep(10000);
    }

    for (int i = 0; i < 6; i++) {
        //iolinker.link(21 + i*2, 13 + i); /* Link Arduino 1 to LCD */
        usleep(10000);
    }

    for (int i = 0; i < 6; i++) {
        //iolinker.link(20 + i*2, 13 + i); /* Link Arduino 2 to LCD */
        usleep(10000);
    }

    for (int i = 0; i < 6; i++) {
        iolinker.link(1 + i*2, 13 + i); /* Link Arduino 3 to LCD */
        usleep(10000);
    }

    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 43); /* Servo GND */
    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 45); /* Servo VCC */
    iolinker.setOutput(false, 43);
    iolinker.setOutput(true, 45);
    iolinker.setPinType(IOLinker::IOLINKER_INPUT, 32); /* Arduino0 servo output */
    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 47); /* Servo output */
    iolinker.link(32, 47);

    return 1;
    uint8_t id = iolinker.firstAddress();
    printf("The first slave address is %d, and the chain length from there is %d.\n",
            id, iolinker.chainLength(id));
}

