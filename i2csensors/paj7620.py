"""
PAJ7620 gesture sensor
"""

import smbus2 as smbus
from enum import Enum
import time


class PAJ7620:
    REG_BANK_SEL = 0xEF
    REG_RESULT_L = 0x43
    REG_RESULT_H = 0x44
    REG_GESTURE_SELECT1 = 0x41
    REG_GESTURE_SELECT2 = 0x42

    REG_IDLE_TIME = 0x65
    REG_CURSOR_INT= 0x44
    REG_CURSOR_X_LOW = 0x3B
    REG_CURSOR_X_HIGH = 0x3C
    REG_CURSOR_Y_LOW = 0x3D
    REG_CURSOR_Y_HIGH = 0x3E

    REG_OBJECT_X_LOW = 0xAC
    REG_OBJECT_X_HIGH = 0xAD
    REG_OBJECT_Y_LOW = 0xAE
    REG_OBJECT_Y_HIGH = 0xAF
    REG_OBJECT_SIZE_LOW = 0xB1
    REG_OBJECT_SIZE_HIGH = 0xB2

    IDLE_TIME_FAR_240FPS = 53

    GESTURE_MODE_WRITES = [
        (0xEF, 0x00),  # Bank 0
        (0x41, 0x00),  # Disable interrupts for first 8 gestures
        (0x42, 0x00),  # Disable wave (and other modes') interrupt(s)
        (0x37, 0x07),
        (0x38, 0x17),
        (0x39, 0x06),
        (0x42, 0x01),
        (0x46, 0x2D),
        (0x47, 0x0F),
        (0x48, 0x3C),
        (0x49, 0x00),
        (0x4A, 0x1E),
        (0x4C, 0x22),
        (0x51, 0x10),
        (0x5E, 0x10),
        (0x60, 0x27),
        (0x80, 0x42),
        (0x81, 0x44),
        (0x82, 0x04),
        (0x8B, 0x01),
        (0x90, 0x06),
        (0x95, 0x0A),
        (0x96, 0x0C),
        (0x97, 0x05),
        (0x9A, 0x14),
        (0x9C, 0x3F),
        (0xA5, 0x19),
        (0xCC, 0x19),
        (0xCD, 0x0B),
        (0xCE, 0x13),
        (0xCF, 0x64),
        (0xD0, 0x21),
        (0xEF, 0x01),  # Bank 1
        (0x02, 0x0F),
        (0x03, 0x10),
        (0x04, 0x02),
        (0x25, 0x01),
        (0x27, 0x39),
        (0x28, 0x7F),
        (0x29, 0x08),
        (0x3E, 0xFF),
        (0x5E, 0x3D),
        (0x65, 0x96),  # R_IDLE_TIME LSB - Set sensor speed to 'normal speed' - 120 fps
        (0x67, 0x97),
        (0x69, 0xCD),
        (0x6A, 0x01),
        (0x6D, 0x2C),
        (0x6E, 0x01),
        (0x72, 0x01),
        (0x73, 0x35),
        (0x74, 0x00),  # Set to gesture mode
        (0x77, 0x01),
        (0xEF, 0x00),  # Bank 0
        (0x41, 0xFF),  # Re-enable interrupts for first 8 gestures
        (0x42, 0x01),  # Re-enable interrupts for wave gesture
    ]

    CURSOR_MODE_WRITES = [
        (0xEF, 0x00),  # Set Bank 0
        (0x32, 0x29),  # Default 29
        (0x33, 0x01),  # Default 01 R_PositionFilterStartSizeTh [7:0]
        (0x34, 0x00),  # Default 00 R_PositionFilterStartSizeTh [8]
        (0x35, 0x01),  # Default 01 R_ProcessFilterStartSizeTh [7:0]
        (0x36, 0x00),  # Default 00 R_ProcessFilterStartSizeTh [8]
        (0x37, 0x03),  # Default 09 R_CursorClampLeft [4:0]
        (0x38, 0x1B),  # Default 15 R_CursorClampRight [4:0]
        (0x39, 0x03),  # Default 0A R_CursorClampUp [4:0]
        (0x3A, 0x1B),  # Default 12 R_CursorClampDown [4:0]
        (0x41, 0x00),  # Interrupt enable mask - Should be 00 (disable gestures)
        (0x42, 0x84),  # Interrupt enable mask - Should be 84 (0b 1000 0100)
        (0x8B, 0x01),  # Default 10 R_Cursor_ObjectSizeTh [7:0]
        (0x8C, 0x07),  # Default 07 R_PositionResolution [2:0]
        (0xEF, 0x01),  # Set Bank 1
        (0x04, 0x03),  # Invert X&Y Axes for GUI coordinates
        (0x74, 0x03),  # Enable cursor mode 0 - gesture, 3 - cursor, 5 - proximity
        (0xEF, 0x00),  # Set Bank 0 (parkin)
    ]

    class Gesture(Enum):
        UP = 0
        DOWN = 1
        LEFT = 2
        RIGHT = 3
        PUSH = 4
        PULL = 5
        CLOCKWISE = 6
        ANTI_CLOCKWISE = 7
        WAVE = 8

    def __init__(self, address=0x73):
        self.address = address
        self.bus = smbus.SMBus(1)
        self.bank = 0
        for reg, value in PAJ7620.GESTURE_MODE_WRITES:
            self.bus.write_byte_data(self.address, reg, value)
        self.write_reg(PAJ7620.REG_IDLE_TIME, PAJ7620.IDLE_TIME_FAR_240FPS, bank=1)
        self.is_gesture_mode=True

    def cursor_mode(self):
        for reg, value in PAJ7620.CURSOR_MODE_WRITES:
            self.bus.write_byte_data(self.address, reg, value)
        self.get_gesture() # clear any leftover gesture 
        self.is_gesture_mode=False

    def gesture_mode(self):
        for reg, value in PAJ7620.GESTURE_MODE_WRITES:
            self.bus.write_byte_data(self.address, reg, value)
        self.get_gesture() # clear any leftover gesture 
        self.is_gesture_mode=True

    def get_cursor_x(self):
        if self.is_gesture_mode:
            l=self.read_reg(PAJ7620.REG_OBJECT_X_LOW)
            h=self.read_reg(PAJ7620.REG_OBJECT_X_HIGH)
        else:
            l=self.read_reg(PAJ7620.REG_CURSOR_X_LOW)
            h=self.read_reg(PAJ7620.REG_CURSOR_X_HIGH)
        h = h&0x0f
        return (h<<8)+l

    def get_cursor_y(self):
        if self.is_gesture_mode:
            l=self.read_reg(PAJ7620.REG_OBJECT_Y_LOW)
            h=self.read_reg(PAJ7620.REG_OBJECT_Y_HIGH)
        else:
            l=self.read_reg(PAJ7620.REG_CURSOR_Y_LOW)
            h=self.read_reg(PAJ7620.REG_CURSOR_Y_HIGH)
        h = h&0x0f
        return (h<<8)+l

    def get_cursor_visible(self):
        if self.is_gesture_mode:
            return (self.get_object_size()>10)
        else:
            return ((self.read_reg(PAJ7620.REG_CURSOR_INT)&0x04)!=0)

    def get_object_size(self):
        l=self.read_reg(PAJ7620.REG_OBJECT_SIZE_LOW)
        h=self.read_reg(PAJ7620.REG_OBJECT_SIZE_HIGH)
        h = h&0x0f
        return (h<<8)+l

    def _set_bank(self, bank):
        """PAJ7620 has two banks of registers,
        which are switched with the bank selection
        register"""
        self.bus.write_byte_data(self.address, PAJ7620.REG_BANK_SEL, (bank & 1))
        self.bank = bank

    def get_gesture(self):
        gesture_result = self.read_reg(PAJ7620.REG_RESULT_L)
        for x in range(8):
            if gesture_result & (1 << x):
                return PAJ7620.Gesture(x)
        return None

    def read_reg(self, reg, bank=0):
        if bank != self.bank:
            self._set_bank(bank)
        return self.bus.read_byte_data(self.address, reg)

    def write_reg(self, reg, value, bank=0):
        if bank != self.bank:
            self._set_bank(bank)
        return self.bus.write_byte_data(self.address, reg, value)


if __name__ == "__main__":
    p = PAJ7620()
#    p.cursor_mode()
    while True:
        time.sleep(0.1)
        print(p.get_cursor_x(),p.get_cursor_y(),p.get_cursor_visible(),p.get_object_size())
#        g = p.get_gesture()
#        if g is not None:
#            print(g, type(g))
