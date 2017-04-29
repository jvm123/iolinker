import IOLinker
import binascii
import time

# Connect to iolinker and print version
iolinker = IOLinker.IOLinker()
iolinker.beginSerial()
version = iolinker.version()
val = iolinker.chainLength()
print("Chain length was determined as: '" + str(val) + "'")
val = iolinker.firstAddress()
print("First address was determined as: '" + str(val) + "'")
iolinker.targetAddress(str(unichr(val)))

print("Version was determined as: '0x" + binascii.hexlify(version) + "'")

# Reset schematic and set all pins as high output
iolinker.clearPinFunctions(1, 49)
iolinker.setPinType(iolinker.OUTPUT, 1, 49)
iolinker.setOutput(True, 1, 49)

# Set P2 as input, it's our push button
iolinker.setPinType(iolinker.INPUT, 2)

# Wait for push button press
print("Please press the push button to start!")
while True:
    if iolinker.readInput(2):
        break

print("Starting!")

# Do a PWM animation on our LEDs P4 and P5
while True:
    for i in range(0, 100):
        iolinker.pwm(i, 4)
        iolinker.pwm(100 - i, 5)
        time.sleep(0.01)

    for i in range(0, 100):
        iolinker.pwm(100 - i, 4)
        iolinker.pwm(i, 5)
        time.sleep(0.01)

