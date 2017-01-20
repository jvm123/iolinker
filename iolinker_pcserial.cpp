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

int main(void)
{
    /* Initialize class */
    IOLinker iolinker;

    iolinker.beginSerial();
    iolinker.targetAddress(1);
    iolinker.setOutput(true, 4); // P4 is high

    //iolinker.setPinType(IOLinker::IOLINKER_PULLDOWN, 1); // P1 is a pulldown input
    //iolinker.setPinType(IOLinker::IOLINKER_PULLUP, 2); // P2 is a pullup input
    iolinker.setPinType(IOLinker::IOLINKER_INPUT, 3); // P3 is a tristate input
    iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 4, 64); // P4 to P64 are outputs

    // Don't forget to set pin types first, see TYP section.

    iolinker.setOutput(true, 4); // P4 is high
    iolinker.setOutput(false, 5); // P5 is low
    iolinker.setOutput(true, 6, 48); // P6 to P48 are high

    
    uint8_t s[] = { 0x00, 0xff };
    iolinker.setOutput(s, sizeof(s), 49, 64); // P49 to P56 are low, P57 to P64 are high

    uint8_t id = iolinker.firstAddress();
    printf("The first slave address is %d, and the chain length from there is %d.\n",
            id, iolinker.chainLength(id));
}

