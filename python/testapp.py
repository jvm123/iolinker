import IOLinker
import binascii

iolinker = IOLinker.IOLinker()
iolinker.beginSerial()
iolinker.setPinType(iolinker.OUTPUT, 0, 127)
print("Version was determined as: '0x" + binascii.hexlify(iolinker.version()) + "'")

