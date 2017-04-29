#include <Wire.h>
#include <SPI.h>
#include <IOLinker.h>
IOLinker iolinker;

void setup() {
  /* Connect to iolinker */
  // On Arduino Uno
  /*Serial.begin(IOLINKER_BAUDRATE); // iolinker UART
  iolinker.beginStream(Serial);*/

  // On Arduino Leonardo
  Serial.begin(IOLINKER_BAUDRATE); // Debug output
  Serial1.begin(IOLINKER_BAUDRATE); // iolinker UART
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  iolinker.beginStream(Serial1);

  // Connect with SPI
  //iolinker.beginSPI();

  // Connect with I2C
  //iolinker.beginI2C();

  /* Register an interrupt to be informed about
   * input pin changes */
  iolinker.registerInterrupt(9);

  /* Turn off all pin links */
  iolinker.clearPinFunctions(1, 10);

  /* Set P1-P10 as high output, except P2, which will
   * be an input (our push button) */
  iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 1, 10);
  iolinker.setOutput(true, 1, 10);
  iolinker.setPinType(IOLinker::IOLINKER_INPUT, 2);
}

void loop() {
  /* Retrieve iolinker version number and print it
   * on serial port */
  Serial.print("Version: ");
  Serial.println(iolinker.version(), HEX);
  delay(10); // Short delay after read command, to be safe

  /* Print button state */
  if (iolinker.readInput(2)) {
    Serial.println("P2 is high");
  } else {
    Serial.println("P2 is low");
  }
  delay(10); // Short delay after read command, to be safe
    
  /* Breathing effect for LEDs P4, P5 */
  for (uint8_t i = 0; i < 128; i++) {
    iolinker.pwm(127 - i, 4);
    iolinker.pwm(i, 5);
    delay(10);
  }
  for (uint8_t i = 128; i > 0; i--) {
    iolinker.pwm(128 - i, 4);
    iolinker.pwm(i - 1, 5);
    delay(10);
  }
}

ISR (PCINT0_vect) {
  /* iolinker interrupt! Some input changed state. */
  Serial.println("INT");
}

