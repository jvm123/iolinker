#include <SPI.h>
#include <Wire.h>
#include <IOLinker.h>

IOLinker iolinker;

void resetschematic() {
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
        iolinker.setOutput(false, KEYPAD_COL0 + i*PIN_INTERVAL);

        /* Read in rows */
        //uint8_t s[20];
        //iolinker.readInput(s, sizeof(s), KEYPAD_ROW0,
        //        KEYPAD_ROW0 + (KEYPAD_ROWS-1)*PIN_INTERVAL);

        for (uint8_t k = 0; k < KEYPAD_ROWS; k++) {
            if (iolinker.readInput(KEYPAD_ROW0 + k*PIN_INTERVAL)) {
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
        iolinker.setOutput(true, KEYPAD_COL0 + i*PIN_INTERVAL);
        iolinker.setPinType(IOLinker::IOLINKER_INPUT,
                KEYPAD_COL0 + i*PIN_INTERVAL);
    }

    return '-';
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  iolinker.beginStream(Serial1); // Connect to iolinker chip via UART
  //iolinker.beginSPI(); // Connect to iolinker chip via SPI
  //iolinker.beginI2C(); // Connect to iolinker chip via I2C
  iolinker.targetAddress(0x7f); // Address of the chip to connect to
  resetschematic();
}

void loop() {
  char btn = button_press();
  Serial.print("Button press: ");
  Serial.print(btn);
  Serial.println(".");
  delay(100);
}

