import serial

ser = serial.Serial('COM3', 115200)
print(ser.readline())
ser.write(0XAF)
while True:
    print(ser.readline())
