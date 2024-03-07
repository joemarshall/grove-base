""" Simple keyboard reading module - for reading from keyboard on the python terminal. 

    Useful for e.g. getting ground truth values to go with sensors
"""

import atexit
import fcntl
import sys
import os
import tty
import termios


class RawTerminal(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)
        self.orig_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl | os.O_NONBLOCK)

    def __del__(self):
        self.close()

    def close(self):
        if self.fd is not None:
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl)
            termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)
            self.fd = None


_RAW_TERMINAL = None


def init_raw():
    global _RAW_TERMINAL
    if _RAW_TERMINAL is None:
        _RAW_TERMINAL = RawTerminal(sys.stdin)
        atexit.register(deinit_raw)


def deinit_raw():
    global _RAW_TERMINAL
    if _RAW_TERMINAL is not None:
        _RAW_TERMINAL.close()


def read_key():
    """ Read a single keypress. Returns empty string if no key is pressed """
    init_raw()
    return sys.stdin.read(1)
