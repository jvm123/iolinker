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
  iolinker.pwm(127, 1, 49);
  iolinker.clearPinFunctions(1, 49);
  iolinker.setPinType(IOLinker::IOLINKER_INPUT, 1, 49);
}

int main(void)
{
    uint16_t ver;
    wiringPiSetup();

    //iolinker.beginSerial("/dev/ttyAMA0");
    //iolinker.beginSPI(0);
    iolinker.beginI2C();
    iolinker.targetAddress(0x7f);
    
    if ((ver = iolinker.version()) != 0x131) {
        printf("ERROR: No response from iolinker (version %x).\n", ver);
        return 0;
    } else {
        printf("iolinker is responding (version %x).\n", ver);
    }

    resetschematic();

    /*uint8_t id = iolinker.firstAddress();
    printf("The first slave address is %d, and the chain length from there is %d.\n",
            id, iolinker.chainLength(id));*/

    uint8_t led1 = 1, led4 = 7, button = 2;
    iolinker.setPinType(IOLinker::IOLINKER_INPUT, button);

    if (iolinker.readInput(button)) {
        printf("Input %d is high.\n", button);
    } else {
        printf("Input %d is low.\n", button);
    }

    printf("Press the push button to start!\n");
    while (!iolinker.readInput(button)) {}

    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, led1, led4);

    printf("Set pins %d to %d high.\n", led1, led4);
    iolinker.setOutput(true, led1, led4);
    usleep(100000);

    setbuf(stdout, NULL);
    for (uint8_t j = 0; j < 4 || 1; j++) {
        printf("PWM");
        for (uint8_t i = 0; i < 128; i++) {
            printf(" %d", i);
            iolinker.pwm(i, led4 - 1);
            iolinker.pwm(127 - i, led1, led4 - 2);
            iolinker.pwm(i, led4);
            usleep(5000);
        }
        printf("\n");
        printf("PWM");
        for (uint8_t i = 127; i > 0; i--) {
            printf(" %d", i);
            iolinker.pwm(i, led4 - 1);
            iolinker.pwm(127 - i, led1, led4 - 2);
            iolinker.pwm(i, led4);
            usleep(5000);
        }
        printf("\n");
    }

    /*iolinker.pwm(100, led1, led4);
    printf("Link %d to %d with pin 11.\n", led1, led4);
    iolinker.link(11, led1, led4);
    printf("Sleeping 10s...\n");
    sleep(10);
    printf("Link %d to %d with pin 11.\n", led1, led4);
    iolinker.clearPinFunctions(led1, led4);
    iolinker.link(11, led1, led4);*/
    return 1;
}

