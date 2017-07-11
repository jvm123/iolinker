/**
 * (C) 2016 jInvent Software Development <prog@jinvent.de>
 * MIT License
 * --
 * http://jinvent.de/IOLinker
 **/

/**
 * @file IOLinker_unittest.cpp
 * @author Julian von Mendel
 * @brief Quick test program for IOLinker class for reading in a 4x4 keypad
 **/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "IOLinker.h"

IOLinker iolinker;

void resetschematic() {
    for (uint8_t i = 0; i < 49; i++) {
      iolinker.pwm(127, i);
      iolinker.clearPinFunctions(i);
      iolinker.setPinType(IOLinker::IOLINKER_INPUT, i);
    }
    return;

  iolinker.pwm(127, 1, 49);
  iolinker.clearPinFunctions(1, 49);
  iolinker.setPinType(IOLinker::IOLINKER_INPUT, 1, 49);
}

/** Keypad definitions **/

#define KEYPAD_INVERTX /*< Invert pin direction */
//#define KEYPAD_INVERTY /*< Invert pin direction */
#define PIN_INTERVAL 2 /*< Use every second pin */
#define KEYPAD_COLS 4 /*< Column count */
#define KEYPAD_ROWS 4 /*< Row count */
#define KEYPAD_ROW0 34 /*< Lowest pin the keypad is connected to */
#define KEYPAD_COL0 (KEYPAD_ROW0 + PIN_INTERVAL * KEYPAD_ROWS)

char keypad[KEYPAD_ROWS][KEYPAD_COLS] = {
        { '1', '2', '3', 'A' },
        { '4', '5', '6', 'B' },
        { '7', '8', '9', 'C' },
        { '*', '0', '#', 'D' } }; /*< Keypad buttons */

/*! \brief Read in the currently pressed keypad button
    \return The button character is returned, or '-' if no button press
        is detected
*/
char button_press() {
    /* Set all pins as pulldown input */
    iolinker.setPinType(IOLinker::IOLINKER_INPUT, KEYPAD_ROW0,
          KEYPAD_ROW0 + KEYPAD_ROWS * PIN_INTERVAL);
    iolinker.setPinType(IOLinker::IOLINKER_INPUT, KEYPAD_COL0,
          KEYPAD_COL0 + KEYPAD_COLS * PIN_INTERVAL);

    /* Cycle through columns */
    for (uint8_t i = 0; i < KEYPAD_COLS; i++) {
        /* One column is a high output, all other columns are
           configured as input */
        iolinker.setPinType(IOLinker::IOLINKER_OUTPUT,
                KEYPAD_COL0 + i*PIN_INTERVAL);
        iolinker.setOutput(true, KEYPAD_COL0 + i*PIN_INTERVAL);

        /* Read in rows */
        //uint8_t s[20];
        //iolinker.readInput(s, sizeof(s), KEYPAD_ROW0,
        //        KEYPAD_ROW0 + (KEYPAD_ROWS-1)*PIN_INTERVAL);

        for (uint8_t k = 0; k < KEYPAD_ROWS; k++) {
            if (!iolinker.readInput(KEYPAD_ROW0 + k*PIN_INTERVAL)) {
                continue;
            }
            //printf("%d high, %d, %d\n", KEYPAD_ROW0 + k*PIN_INTERVAL, i, k);

            /* Row input is high, i.e. we found a pressed key */
#ifdef KEYPAD_INVERTY
            uint8_t tmp = k;
            k = i;
            i = tmp;
#endif
#ifndef KEYPAD_INVERTX
            return keypad[i][k];
#else
            return keypad[KEYPAD_ROWS - 1 - i][KEYPAD_COLS - 1 - k];
#endif
        }

        /* Reset column to input state */
        iolinker.setPinType(IOLinker::IOLINKER_INPUT,
                KEYPAD_COL0 + i*PIN_INTERVAL);
        usleep(10000);
    }

    return '-';
}

int main(void)
{
    iolinker.beginSerial();
    iolinker.targetAddress(0x7f);
    if (iolinker.version() == 0) {
        printf("ERROR: No response from iolinker (version %x).\n",
                iolinker.version());
        //return 0;
    }
    printf("iolinker is responding.\n");
    resetschematic();

    for (uint16_t i = 0; i < 49; i++) {
        if (iolinker.readInput(i)) {
            printf("%d high\n", i);
        }
    }
    
    printf("Waiting for key press...\n");
	while (1) {
        char key;
        if ((key = button_press()) != '-') {
            printf("Button press: %c.\n", key);
        }
    }
}

