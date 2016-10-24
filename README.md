# iolinker

Web: http://jinvent.de/iolinker

## The iolinker Chips

Every iolinker chip functions as a dynamically configurable IO matrix. Its main functionality, besides IO extension, is to dynamically set up a matrix of GPIO connections, that allow direct (or inverted) pass-through of high-frequency signals. Circuits can thereby be configured and programmed on the fly. There are UART / SPI / I2C versions that allow for easy integration of up to 127 chips connected in parallel. Pro versions also allow for PWM signal output.

You can find the datasheet at http://jinvent.de/resources/iolinker_datasheet.pdf.

## The iolinker Library

The iolinker library allows easy interfacing of the iolinker chips, on a PC, a Raspberry Pi or on an Arduino. It works with UART, SPI and I2C chips.

## Example usage

Note that wherever the examples use *Serial.println()*, the code may seem Arduino specific. But you could as well change the appropriate lines into a *printf()* statement for Raspberry or PC usage.

### Initialization on PC and Raspberry

Setting up the serial interface on a Raspberry or PC, to communicate with an UART iolinker chip:

```c++
IOlinker.beginSerial("/dev/ttyACM0"); // or ttyUSB0 or the like
```

Setting up the SPI interface on a Raspberry, to communicate with an SPI iolinker chip:
```c++
IOlinker.beginSPI(0); // Channel 0
```

Setting up the I2C interface, for I2C chips:
```c++
IOlinker.beginI2C();
```

### Initialization on an Arduino:

Setting up the serial interface to communicate with UART iolinker chips:

```c++
IOlinker.beginStream(Serial);
```

Setting up the SPI interface, for SPI chips:
```c++
IOlinker.beginSPI();
```

Setting up the I2C interface, for I2C chips:
```c++
IOlinker.beginI2C();
```

### Slave address set-up

You can connect up to 127 chips in parallel. Their addresses are set through their hardware slave address port (IN1 to IN7).

When using multiple, address collision obviously has to be avoided. I suggest you start with address 1 and go up from there.

```c++
IOlinker.targetAddress(1);
```

When using only one chip, you can cheat if you aren't sure about its slave address:

```c++
/* Walk through possible slave addresses and use first one that works */
IOlinker.targetAddress(IOlinker.firstAddress());
Serial.print("The first slave address is ");
Serial.println(IOlinker.firstAddress(), DEC);
```

### TYP: Changing pin types

By default, all pins on the iolinker chip are Tristate (open collector) inputs. Change those easily:

```c++
IOlinker.setPinType(IOLINKER_PULLDOWN, 1); // P1 is a pulldown input
IOlinker.setPinType(IOLINKER_PULLUP, 2); // P2 is a pullup input
IOlinker.setPinType(IOLINKER_INPUT, 3); // P3 is a tristate input
IOlinker.setPinType(IOLINKER_OUTPUT, 4, 64); // P4 to P64 are outputs
```

Outputs are low, until their output value is changed with the SET command.

### SET: Change pin output value

```c++
// Don't forget to set pin types first, see TYP section.

IOlinker.setOutput(true, 4); // P4 is high
IOlinker.setOutput(false, 5); // P5 is low
IOlinker.setOutput(true, 6, 48); // P6 to P48 are high

uint8_t s[] = { 0x00, 0xff };
IOlinker.setOutput(s, sizeof(s), 49, 64); // P49 to P56 are low, P57 to P64 are high
```

### REA: Read pin states

The *readInput()* method allows to determine pin states. On an Arduino this would work:

```c++
// Don't forget to set pin types first, see TYP section.

if (IOlinker.readInput(1)) {
    Serial.println("P1 is high!");
} else {
    Serial.println("P1 is low!");
}
```

### LNK: Link pins

Link two pins, and configure your circuit dynamically! The pin link allows to transfer MHz signals from one iolinker pin to another.

```c++
// Don't forget to set pin types first, see TYP section.

IOlinker.link(1, 7); // P7 outputs all values from P1
IOlinker.link(1, IOlinker.invert(8)); // P8 outputs all values from
                                      // P1's inverted pin state
IOlinker.link(1, 9, 11); // P9 to 11 output all values from P1
IOlinker.link(IOLINKER_VIRTUAL_CLK256, 12); // P12 outputs the internal iolinker clock divided by 256
```

For more virtual pins, check out the iolinker datasheet, section "Pin addresses".

### PWM: Activate pulse width modulation

```c++
// Don't forget to set pin types first, see TYP section.

IOlinker.pwm(127, 6); // P6 is 100% on -- equals normal high state
IOlinker.pwm(0, 7); // P7 is 0% on
IOlinker.pwm(64, 8, 15); // P8 to P15 are ~50.4% on

IOlinker.setOutput(true, 6, 15); // PWM output is only active when the pins are set to output type, and their output state is set to high

IOlinker.setOutput(false, 12); // Turn PWM output on P12 back off, just for demonstration
```

### Change PWM period

PWM period can only be set for all pins simultaneously.

```c++
IOlinker.pwmPeriod(0); // Shortest period
IOlinker.pwmPeriod(127); // Longest period
```

### Synchronizing pin updates

If you are updating a lot of pins individually and want the effect to be simultaneous, the iolinker chips allow to buffer pin settings. For that purpose, you first use the SYN command with *IOlinker.syncOutputsToBuffer()*, activate the buffer with *IOlinker.buffer(true)*, then run your pin update commands, and then trigger the simultaneous pin update using the TRG command with *IOlinker.syncBufferToOutputs()*.

```c++
// Prepare buffered pin update
IOlinker.syncOutputsToBuffer();
IOlinker.buffer(true);

// Your commands
IOlinker.pwm(13, 8); // P8 will be ~10.23% on
IOlinker.setOutput(true, 9); // P9 will be high
IOlinker.setOutput(false, 10); // P10 will be low
IOlinker.setPinType(IOLINKER_PULLDOWN, 2); // P2 will be a pulldown input

/* Note that the last four commands did not have any effect on the
   actual pin states yet! We activate them all at once now: */

IOlinker.syncBufferToOutputs(); // Simultaneous pin update!
```

### Data safety: Checksums and status codes

If it is vital for you that your changes are implemented correctly, you can activate CRC7 checksums and verify status codes of return messages. Checksums are activated and verified both ways.

```c++
// Activate CRC for all future commands
IOlinker.crc(true);

// ...your command for changing output states or reading input
// states, as shown above, e.g.
IOlinker.setPinType(IOLINKER_PULLDOWN, 2); // P2 will be a pulldown input

// Verify the status code of the last command
select (IOlinker.statusCode()) {
    case IOLINKER_STATUS_SUCCESS:
        Serial.println("Success! Congratulations.");
        break;
    case IOLINKER_ERROR_CRC:
        Serial.println("CRC error. Please check your hardware!");
        break;
    case IOLINKER_ERROR_NOREPLY:
        Serial.println("No reply. Please check your wiring and initialization!");
        break;
    case IOLINKER_ERROR_INTERFACE:
        Serial.println("Your interface setup was not correct. Please fix :)");
        break;
    default:
        Serial.println("Undefined error from chip. Please make sure that the message you sent made sense.");
}
```

### Reset chip state

```c++
IOlinker.reset();
```

## Dependencies

* Raspberry
  * WiringPi (http://wiringpi.com/download-and-install/)

