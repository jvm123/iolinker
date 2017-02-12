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
  iolinker.pwm(127, 0, 48);
  iolinker.clearPinFunctions(0, 48);
  iolinker.setPinType(IOLinker::IOLINKER_INPUT, 0, 48);
}

#define PIN_INTERVAL 2
#define KEYPAD_INVERT
#define KEYPAD_COLS 4
#define KEYPAD_ROWS 4
#define KEYPAD_COL0 24
#define KEYPAD_COLL 30
#define KEYPAD_ROW0 16
#define KEYPAD_ROWL 22
char keypad[KEYPAD_ROWS][KEYPAD_COLS] = {
        { '1', '2', '3', 'A' },
        { '4', '5', '6', 'B' },
        { '7', '8', '9', 'C' },
        { '*', '0', '#', 'D' } };

void schematicKeyPad() {
  iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, KEYPAD_COL0,
          KEYPAD_COL0 + KEYPAD_COLS * PIN_INTERVAL);
  iolinker.setPinType(IOLinker::IOLINKER_INPUT, KEYPAD_ROW0,
          KEYPAD_ROWL + KEYPAD_COLS * PIN_INTERVAL);
}

char _button_press() {
    int row = -1, col = -1, rows = 0;
    char btn;
  
    for (uint8_t i = 0; i < KEYPAD_COLS; i++) {
        iolinker.setPinType(IOLinker::IOLINKER_OUTPUT,
                KEYPAD_COL0 + i*PIN_INTERVAL);
        iolinker.setOutput(false, KEYPAD_COL0 + i*PIN_INTERVAL);
    
        uint8_t btn_zero = 0;
        for (uint8_t k = 0; k < KEYPAD_ROWS; k++) {
            if (!iolinker.readInput(KEYPAD_ROW0 + k*PIN_INTERVAL)) {
                btn_zero++;
            } else {
                col = k;
            }
            usleep(100);
        }

        iolinker.setOutput(true, KEYPAD_COL0 + i*PIN_INTERVAL);

        if (btn_zero == 4) {
            row = i;
            rows++;
            continue;
        }
    }

    if (rows == 4) {
        btn = '-';
    } else {
#ifndef KEYPAD_INVERT
        btn = keypad[row][col];
#else
        btn = keypad[3 - row][3 - col];
#endif
    }
    
    return btn;
}

/*! \brief Filtered key presses */
char button_press() {
    static char val = '-';
    char read = _button_press();

    char read2 = _button_press();
    if (read != read2) {
        return '-';
    }
    
    if (read != '-' && val == '-') {
        val = read;
        return read;
    }

    if (read == '-') {
        val = '-';
    }
    return '-';
}

int main(void)
{
    iolinker.beginSerial();
    iolinker.targetAddress(0x7f);
    if (iolinker.version() != 0x131) {
        printf("ERROR: No response from iolinker (version %x).\n",
                iolinker.version());
        return 0;
    }
    printf("iolinker is responding.\n");
    printf("Waiting for key press...\n");

    resetschematic();
    schematicKeyPad();
	while (1) {
        char key;
        if ((key = button_press()) != '-') {
            printf("Button press: %c.\n", key);
        }
    }
}

