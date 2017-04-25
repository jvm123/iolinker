# IOLinker Python Library

This is still under development and hardly tested. 

Dependencies: Python and pyserial.

## Usage

'''
import IOLinker
import binascii

iolinker = IOLinker.IOLinker()
iolinker.beginSerial()
if not iolinker.available():
    print("No reply from iolinker chip!")
    exit(0)

print("Version : '0x" + binascii.hexlify(iolinker.version()) + "'")

iolinker.setPinType(iolinker.OUTPUT, 1, 3)
iolinker.setOutput(True, 1, 3) # P1 to P3 turn high
iolinker.pwm(50, 3) # 50% PWM for P3

# Read input pin P5
iolinker.setPinType(iolinker.INPUT, 5)
if iolinker.readInput(5):
    print("P5 is high!")

# Link P2 to P5
iolinker.link(5, 2)
'''

