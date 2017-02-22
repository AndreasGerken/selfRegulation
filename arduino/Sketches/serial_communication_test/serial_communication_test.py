import serial
import time
import numpy as np
import pickle
import struct

current_milli_time = lambda: int(round(time.time() * 1000))


ser = serial.Serial('/dev/ttyUSB1', 115200)
if(ser):
    print("Serial connection established")
else:
    print("Serial connection failed")

startTime = current_milli_time()
sent1 = sent2 = sent3 = arduReady = False
c = np.reshape(np.arange(4),(2,2))

while(1):
    while(ser.in_waiting):
        newLine = ser.readline().decode()
        if("Arduino is ready" in newLine):
            arduReady = True
        print("Ardu:\t", str.strip(newLine))

    runtime = current_milli_time() - startTime
    

    if(arduReady):
        if(runtime > 100000 and not sent1):
            ser.write("ABCDEFGHIJK\n".encode())
            print("data1 sent")
            sent1 = True

        if(runtime > 100000 and not sent2):
            ser.write("CBEGASGASDGASDG\n".encode())
            print("data2 sent")   
            sent2 = True

        if(runtime > 1000 and not sent3):
            ser.write('C'.encode())

            for i in np.nditer(c):
                print(struct.pack('f',i))
                ser.write(struct.pack('f',i))   
            print("data3 sent")  
            sent3 = True

        if(runtime > 12000):
            exit

    #time.sleep(1)
