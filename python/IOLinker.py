"""
The IOLinker Python class, to connect serially to a compatible IOLinker chip
"""
# (C) 2017 Julian von Mendel <prog@jinvent.de>
# MIT License

import serial
import binascii
import time

class IOLinker:
    IOLINKER_BITMASK_RW_BIT = (1 << 6)

    IOLINKER_CMD_VER = int("01", 16) | IOLINKER_BITMASK_RW_BIT
    IOLINKER_CMD_TYP = int("02", 16)
    IOLINKER_CMD_REA = int("07", 16) | IOLINKER_BITMASK_RW_BIT
    IOLINKER_CMD_SET = int("03", 16)
    IOLINKER_CMD_SYN = int("08", 16)
    IOLINKER_CMD_TRG = int("09", 16)
    IOLINKER_CMD_LNK = int("04", 16)
    IOLINKER_CMD_PWM = int("05", 16)
    IOLINKER_CMD_PER = int("06", 16)
    IOLINKER_CMD_CLR = int("0a", 16)
    IOLINKER_CMD_RST = int("0f", 16)

    INPUT = 0
    OUTPUT = 3

    def __init__(self, targetaddr = b"\x7f"):
        self.ser = False
        self.targetAddress(targetaddr)
        self.cmdbyte = int("80", 16)

    """
    Sets the target address of the iolinker chip to connect to
    """
    def targetAddress(self, targetaddr = b"\x7f"):
        self.targetaddr = targetaddr

    def argData(self, byte):
        return byte & 0x7f

    def pinDistance(self, start, end):
        if end == 0:
            return 0
        if start >= end:
            return start - end
        return end - start

    """
    Connects to a serial port
    """
    def beginSerial(self, port_ = "/dev/ttyUSB0"):
        try:
            self.ser = serial.Serial(
                port = port_,
                baudrate = 115200,
            )
            self.ser.isOpen()
            return True
        except serial.SerialException:
            raise ValueError("ERROR: Serial port " + port_ + " couldn't be opened")
            return False

    def sendAndRead(self, sendmsg, readlength = 0):
        if self.ser == False:
            raise ValueError("Serial port hasn't been opened!")

        self.ser.write(self.targetaddr)
        sendmsg[0] |= self.cmdbyte
        self.ser.write(sendmsg)

        if readlength == 0:
            return True

        read = [ ]
        time.sleep(0.01)
        while self.ser.inWaiting() > 0:
            c = self.ser.read(1)
#print("Got byte: " + binascii.hexlify(c))
            read.append(c)
        return bytearray(read)

    """
    Requests the version from the chip
    """
    def version(self):
        buf = [ self.IOLINKER_CMD_VER ]
        data = self.sendAndRead(bytearray(buf), 2)
        return data

    """
    Determines if the chip is replying
    """
    def available(self):
        version = self.version()
        return (len(version) > 0 and version != 0)

    """
    Sets pin type for a pin range
    """
    def setPinType(self, pin_type, pin_start, pin_end = 0):
        buf = [ self.IOLINKER_CMD_TYP,
            self.argData(pin_start), self.argData(pin_start >> 7),
            self.argData(pin_end), self.argData(pin_end >> 7),
            self.argData(pin_type) ]
        return self.sendAndRead(bytearray(buf), 0)

    """
    Reads register from iolinker chip
    """
    def readRegister(self, addr):
        buf = [ self.IOLINKER_CMD_REA,
            b"\x27", # read register code
            self.argData(addr), 0, 0 ]
        data = self.sendAndRead(bytearray(buf), 1)
        return data

    """
    Read pin state
    """
    def readInput(self, pin):
        buf = [ self.IOLINKER_CMD_REA, 
            self.argData(pin), self.argData(pin >> 7),
            0, 0 ] # single pin, so second pin number in range is 0
        data = self.sendAndRead(bytearray(buf), 1)
       
        if len(data) == 0 or not data[0] == int("40", 16):
            return False

        return True

#    """
#    Read several pin states
#    """
#   def readInput(self, pin_start, pin_end):
#        buf = [ self.IOLINKER_CMD_REA, 
#            self.argData(pin_start), self.argData(pin_start >> 7),
#            self.argData(pin_end), self.argData(pin_end >> 7) ]
#        bytecount = round(self.pinDistance(pin_start, pin_end) + 7) / 7)
#        data = self.sendAndRead(bytearray(buf), bytecount)
#        # TODO: process data and return

    """
    Set pin state for a pin range
    """
    def setOutput(self, state, pin_start, pin_end):
        if state:
            byte = b"\x7f"
        else:
            byte = 0

        buf = [ self.IOLINKER_CMD_SET, 
            self.argData(pin_start), self.argData(pin_start >> 7),
            self.argData(pin_end), self.argData(pin_end >> 7), byte ]
        return self.sendAndRead(bytearray(buf), 0)

#    """
#    Set pin state for a range of pins
#    """
#    def setOutput(self, states, pin_start, pin_end):
#        buf = [ self.IOLINKER_CMD_SET, 
#            self.argData(pin_start), self.argData(pin_start >> 7),
#            self.argData(pin_end), self.argData(pin_end >> 7), ]
#        # TODO: add states to buf
#        return self.sendAndRead(bytearray(buf), 0)

    """
    Start buffering
    """
    def beginBuffering(self):
        buf = [ self.IOLINKER_CMD_SYN ]
        return self.sendAndRead(bytearray(buf), 0)

    """
    Execute buffer and end buffering
    """
    def executeBuffer(self):
        buf = [ self.IOLINKER_CMD_TRG ]
        return self.sendAndRead(bytearray(buf), 0)

    """
    Link pin range to target pin
    """
    def link(self, target_pin, pin_start, pin_end = 0):
        buf = [ self.IOLINKER_CMD_LNK, 
            self.argData(pin_start), self.argData(pin_start >> 7),
            self.argData(pin_end), self.argData(pin_end >> 7),
            self.argData(target_pin), self.argData(target_pin >> 7), ]
        return self.sendAndRead(bytearray(buf), 0)

    """
    Set PWM value
    """
    def pwm(self, pwm_r, pin_start, pin_end = 0):
        buf = [ self.IOLINKER_CMD_PWM, 
            self.argData(pin_start), self.argData(pin_start >> 7),
            self.argData(pin_end), self.argData(pin_end >> 7),
            self.argData(int(round(float(pwm_r) / 100 * 127))) ]
        return self.sendAndRead(bytearray(buf), 0)

#    """
#    Set PWM period
#    """
#    def pwmPeriod(self, per):
#        buf = [ self.IOLINKER_CMD_PER, self.argData(per) ]
#        return self.sendAndRead(buf)
    

    """
    Clear pin links for pin range
    """
    def clearPinFunctions(self, pin_start, pin_end):
        buf = [ self.IOLINKER_CMD_CLR, 
            self.argData(pin_start), self.argData(pin_start >> 7),
            self.argData(pin_end), self.argData(pin_end >> 7) ]
        return self.sendAndRead(bytearray(buf))

    """
    Reset volatile chip settings
    """
    def reset(self):
        buf = [ self.IOLINKER_CMD_RST ]
        return self.sendAndRead(bytearray(buf))


    """
    Determine first target chip address that replies to messages
    """
    def firstAddress(self):
        for i in range(0, 127):
            self.targetAddress(str(unichr(i)))
            if self.available():
                return i
        return 127

    """
    Determine number of chips that are available with sequential target
    addresses
    """
    def chainLength(self, start = 0):
        chainlen = 0
        bak = self.targetaddr

        while True:
            self.targetAddress(str(unichr(start + chainlen)))
            chainlen += 1
            if (start + chainlen) > 127 or not self.available():
                break
        self.targetAddress(bak)
        return chainlen - 1

