import numpy as np
import mmap
import RPi.GPIO as GPIO
import os
import time

class FastDHT11:
    gpiomap=None
    def __init__(self):
        if FastDHT11.gpiomap==None:
            f = os.open("/dev/gpiomem",os.O_RDWR|os.RWF_SYNC|os.O_CLOEXEC)
            print(f)
            mapdata = mmap.mmap(f,4*1024,flags=mmap.MAP_SHARED,access=mmap.ACCESS_READ,offset=0x00200000)
            FastDHT11.gpiomap =memoryview(mapdata).cast("I")

    def readFast(self,pin,reads):
        mapData=FastDHT11.gpiomap
        dataOut= np.zeros(shape=(reads,),dtype=np.uint32)
        pinlevel_read=13+pin//32
        pinShift = pin&31
        pinAnd = 1<<pinShift
        for c in range(reads):
             dataOut[c]=mapData[pinlevel_read]
        for c in range(reads):
             dataOut[c]= dataOut[c]>>pinShift & 1


    def readTransitionsUntilStable(self,pin,stableReads):
        transitions=[]
        mapData=FastDHT11.gpiomap
        pinlevel_read=13+pin//32
        pinShift = pin&31
        pinAnd = 1<<pinShift
        stopPos=stableReads
        totalCount=0
        lastVal =-1
        levelCount=0
        while totalCount<stopPos:
            totalCount+=1
            levelCount+=1
            thisVal=(mapData[pinlevel_read]&pinAnd)
            if lastVal!=thisVal:
                stopPos=stableReads+totalCount
                transitions.append(((lastVal>>pinShift)&1,levelCount))
                lastVal=thisVal
                levelCount=0
        return transitions

    def read(self,pin):
        for retries in range(5):
#            print("Retry:",retries)
            time.sleep(0.001)
            t1=time.monotonic_ns()
            GPIO.setup(pin,GPIO.OUT)
            GPIO.output(pin,1)
            time.sleep(0.05) 
            GPIO.output(pin,0)
            time.sleep(0.02) 
            GPIO.setup(pin,GPIO.IN, GPIO.PUD_UP)
            transitions= self.readTransitionsUntilStable(pin,10000)
            if len(transitions)<80:
                continue
            # low time is the median time of a low bit
            zero_lengths=[x[1] for x in transitions if x[0]==0 ]
            one_bits=[x[1] for x in transitions if x[0]==1 ]
#            print(zero_lengths,one_bits)
            low_time = sorted(zero_lengths)[len(zero_lengths)//2]
            high_cutoff = max(one_bits[-20:])*0.5
#            print(low_time,high_cutoff)
            bits=[]
            for val,dur in transitions:
                if val==1:
                    if dur>high_cutoff:
                        bits.append(True)
                    else:
                        bits.append(False)
                if val==0 and dur>low_time*1.5:
                    bits.append(False)
                    
 #           print(bits,len(bits))        
            bits=bits[-40:]
 #           print(bits,len(bits))        
            output_data=[]
            curByte=0
            for c,x in enumerate(bits):
                curByte<<=1
                if x:
                    curByte|=1
                if c&7==7:
                    output_data.append(curByte)
                    curByte=0
            if sum(output_data[0:4]) &0xff != output_data[4]:
                print("BAD checksum:",output_data)
                time.sleep(0.1)
            else:
                humidity = output_data[0]+float(output_data[1])/10
                temp = output_data[2]+float(output_data[3])/10
                return humidity,temp
            print(output_data,sum(output_data[0:4])&0xff)
        print("DHT read failed")
        return (None,None)
        

GPIO.setmode(GPIO.BCM)
GPIO.setup(26,GPIO.IN, GPIO.PUD_UP)
fg = FastDHT11()
while True:
#    print(GPIO.input(26))
    print(fg.read(22))
#    fg.readFast(26,10)
    time.sleep(0.5)

