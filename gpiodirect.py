import numpy as np
import mmap
import os
import time
import ctypes
import datetime


import gpiod
from gpiod.line import Direction,Value,Bias,Edge


class FastDHT11:
    def __init__(self):
        # gpiochip0 points to gpiochip0 on newer version of Raspberry pi os, even
        # if they don't actually have it. On Pi5, gpiochip4 is the external GPIOs
        self.chip = gpiod.Chip("/dev/gpiochip4")

    def read(self, pin):
        for retries in range(10):
            with self.chip.request_lines(
                config={
                    pin: gpiod.LineSettings(
                        direction=Direction.OUTPUT, output_value=Value.ACTIVE
                    )
                },
                event_buffer_size=256,
                consumer="dht11",
            ) as req:
                time.sleep(0.01)
                req.set_value(pin,Value.INACTIVE)
                time.sleep(0.01)
                req.reconfigure_lines({pin:gpiod.LineSettings(
                        direction=Direction.INPUT, bias=Bias.PULL_UP,edge_detection=Edge.FALLING)})
                time.sleep(0.03)
                if req.wait_edge_events(0.01):
                    events=req.read_edge_events(128)
                    pulses = [ (e2.event_type,e2.timestamp_ns - e.timestamp_ns) for e2,e in zip(events[1:],events[:-1])]
                    high_pulses = [p[1]//1000 for p in pulses]
                    if high_pulses[0]>150:
                        high_pulses=high_pulses[1:]
                    high_pulses = [1 if x>100 else 0 for x in high_pulses]
                    while len(high_pulses)<40:
                        high_pulses.append(0)
                    output_data=[]
                    curByte=0
                    for c, x in enumerate(high_pulses[-40:]):
                        curByte <<= 1
                        if x:
                            curByte |= 1
                        if c & 7 == 7:
                            output_data.append(curByte)
                            curByte = 0
                    if (sum(output_data[0:4])&0xff)==output_data[4]:
                        humidity = output_data[0] + float(output_data[1]) / 10
                        temp = output_data[2] + float(output_data[3]) / 10
                        return humidity, temp
                    time.sleep(0.05)

if __name__=="__main__":
    fg = FastDHT11()
    while True:
        print(fg.read(5))
    #    time.sleep(0.01)
