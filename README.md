# iolinker

Web: http://jinvent.de/iolinker

## The iolinker Chips

Every iolinker chip functions as a dynamically configurable IO matrix. Its main functionality, besides IO extension, is to dynamically set up a matrix of GPIO connections, that allow direct (or inverted) pass-through of high-frequency signals. Circuits can thereby be configured and programmed on the fly. There are UART / SPI / I2C versions that allow for easy integration of up to 127 chips connected in parallel. Pro versions also allow for PWM signal output.

You can find the datasheet at http://jinvent.de/resources/iolinker_datasheet.pdf.

## The iolinker Library

The iolinker library allows easy interfacing of the iolinker chips, on a PC, a Raspberry Pi or on an Arduino. It works with UART, SPI and I2C chips.

## Install

To use the library on a PC, simply switch into the iolinker library directory and use *make clean test* to compile and run the unit tests, or *make clean pcserial* to compile the serial test program.

To use the library on an Arduino, download the IOLinker.zip file, and use Sketch -> Import Library -> Add Library. Then paste below code examples into your main program.

To compile it on a Raspberry Pi, please install the WiringPi library first (http://wiringpi.com/download-and-install/). Also run *sudo apt-get install g++-4.8*. *make clean pi* in the iolinker library directory is enough to compile a test program afterwards. Run it with *sudo ./iolinker_pi*.

## Example usage

Note that wherever the examples use *Serial.println()*, the code may seem Arduino specific. But you could as well change the appropriate lines into a *printf()* statement for Raspberry or PC usage.

### Initialization on PC and Raspberry

Setting up the serial interface on a Raspberry or PC, to communicate with an UART iolinker chip:

```c++
iolinker.beginSerial("/dev/ttyAMA0"); // or ttyUSB0 or the like
```

Setting up the SPI interface on a Raspberry, to communicate with an SPI iolinker chip:
```c++
iolinker.beginSPI(0); // Channel 0
```

Setting up the I2C interface, for I2C chips:
```c++
iolinker.beginI2C();
```

### Initialization on an Arduino:

Setting up the serial interface to communicate with UART iolinker chips:

```c++
Serial.begin(IOLINKER_BAUDRATE); // Use Serial1 on Arduino Leonardo!
while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
}
iolinker.beginStream(Serial);
```

Setting up the SPI interface, for SPI chips:
```c++
iolinker.beginSPI();
```

Setting up the I2C interface, for I2C chips:
```c++
iolinker.beginI2C();
```

### Slave address set-up

You can connect up to 127 chips in parallel. Their addresses are set through their hardware slave address port (IN1 to IN7).

When using multiple, address collision obviously has to be avoided. I suggest you start with address 1 and go up from there.

```c++
iolinker.targetAddress(1);
```

When using only one chip, you can cheat if you aren't sure about its slave address:

```c++
/* Walk through possible slave addresses and use first one that works */
iolinker.targetAddress(iolinker.firstAddress());
Serial.print("The first slave address is ");
Serial.println(iolinker.firstAddress(), DEC);
```

### TYP: Changing pin types

By default, all pins on the iolinker chip are Tristate (open collector) inputs. Change those easily:

```c++
iolinker.setPinType(IOLinker::IOLINKER_PULLDOWN, 1); // P1 is a pulldown input
iolinker.setPinType(IOLinker::IOLINKER_PULLUP, 2); // P2 is a pullup input
iolinker.setPinType(IOLinker::IOLINKER_INPUT, 3); // P3 is a tristate input
iolinker.setPinType(IOLinker::IOLINKER_OUTPUT, 4, 64); // P4 to P64 are outputs
```

Outputs are low, until their output value is changed with the SET command.

### SET: Change pin output value

```c++
// Don't forget to set pin types first, see TYP section.

iolinker.setOutput(true, 4); // P4 is high
iolinker.setOutput(false, 5); // P5 is low
iolinker.setOutput(true, 6, 48); // P6 to P48 are high

uint8_t s[] = { 0x00, 0xff };
iolinker.setOutput(s, sizeof(s), 49, 64); // P49 to P56 are low, P57 to P64 are high
```

### REA: Read pin states

The *readInput()* method allows to determine pin states. On an Arduino this would work:

```c++
// Don't forget to set pin types first, see TYP section.

if (iolinker.readInput(1)) {
    Serial.println("P1 is high!");
} else {
    Serial.println("P1 is low!");
}
```

### LNK: Link pins

Link two pins, and configure your circuit dynamically! The pin link allows to transfer MHz signals from one iolinker pin to another.

```c++
// Don't forget to set pin types first, see TYP section.

iolinker.link(1, 7); // P7 outputs all values from P1
iolinker.link(1, iolinker.invert(8)); // P8 outputs all values from
                                      // P1's inverted pin state
iolinker.link(1, 9, 11); // P9 to 11 output all values from P1
iolinker.link(IOLinker::IOLINKER_VIRTUAL_CLK256, 12); // P12 outputs the internal iolinker clock divided by 256
```

For more virtual pins, check out the iolinker datasheet, section "Pin addresses".

### PWM: Activate pulse width modulation

```c++
// Don't forget to set pin types first, see TYP section.

iolinker.pwm(127, 6); // P6 is 100% on -- equals normal high state
iolinker.pwm(0, 7); // P7 is 0% on
iolinker.pwm(64, 8, 15); // P8 to P15 are ~50.4% on

iolinker.setOutput(true, 6, 15); // PWM output is only active when the pins are set to output type, and their output state is set to high

iolinker.setOutput(false, 12); // Turn PWM output on P12 back off, just for demonstration
```

### Change PWM period

PWM period can only be set for all pins simultaneously.

```c++
iolinker.pwmPeriod(0); // Shortest period
iolinker.pwmPeriod(127); // Longest period
```

### Synchronizing pin updates

If you are updating a lot of pins individually and want the effect to be simultaneous, the iolinker chips allow to buffer pin settings. For that purpose, you first use the SYN command with *iolinker.syncOutputsToBuffer()*, activate the buffer with *iolinker.buffer(true)*, then run your pin update commands, and then trigger the simultaneous pin update using the TRG command with *iolinker.syncBufferToOutputs()*.

```c++
// Prepare buffered pin update
iolinker.syncOutputsToBuffer();
iolinker.buffer(true);

// Your commands
iolinker.pwm(13, 8); // P8 will be ~10.23% on
iolinker.setOutput(true, 9); // P9 will be high
iolinker.setOutput(false, 10); // P10 will be low
iolinker.setPinType(IOLinker::IOLINKER_PULLDOWN, 2); // P2 will be a pulldown input

/* Note that the last four commands did not have any effect on the
   actual pin states yet! We activate them all at once now: */

iolinker.syncBufferToOutputs(); // Simultaneous pin update!
```

### CLR: Clear PWM output and link settings for pins

If you want to turn PWM or pin links back off for a pin and reuse it for something else, use the *iolinker.clearPinFunctions(firstpin, lastpin)* method.

```c++
iolinker.clearPinFunctions(1); // Reset settings for P1
iolinker.clearPinFunctions(2, 10); // Reset settings for P2 to P10
```

### Data safety: Checksums and status codes

If it is vital for you that your changes are implemented correctly, you can activate CRC7 checksums and verify status codes of return messages. Checksums are activated and verified both ways.

```c++
// Activate CRC for all future commands
iolinker.crc(true);

// ...your command for changing output states or reading input
// states, as shown above, e.g.
iolinker.setPinType(IOLinker::IOLINKER_PULLDOWN, 2); // P2 will be a pulldown input

// Verify the status code of the last command
select (iolinker.statusCode()) {
    case IOLinker::IOLINKER_STATUS_SUCCESS:
        Serial.println("Success! Congratulations.");
        break;
    case IOLinker::IOLINKER_ERROR_CRC:
        Serial.println("CRC error. Please check your hardware!");
        break;
    case IOLinker::IOLINKER_ERROR_NOREPLY:
        Serial.println("No reply. Please check your wiring and initialization!");
        break;
    case IOLinker::IOLINKER_ERROR_INTERFACE:
        Serial.println("Your interface setup was not correct. Please fix :)");
        break;
    default:
        Serial.println("Undefined error from chip. Please make sure that the message you sent made sense.");
}
```

### Reset chip state

```c++
iolinker.reset();
```

## Dependencies

* Raspberry
  * WiringPi (http://wiringpi.com/download-and-install/)

