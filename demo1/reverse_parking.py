import serial
import time
import sys,tty,termios

# XBee setting
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600, timeout=3)

def time_len(distance) :
    t = distance / 15 # in sec
    return t

d1, d2, face = map(str, input().split())
d1 = int(d1)
d2 = int(d2)

t1 = time_len(d1 + 10)
t2 = time_len(d2 + 15 - 10)

# run d1
print("start\n")
s.write("start\n".encode())
print("/goStraight/run 100 \n")
s.write("/goStraight/run 100 \n".encode())
time.sleep(t1)
print("/stop/run \n")
s.write("/stop/run \n".encode())
# turn
if (face == "west"):
    print("/turn2/run -100 -0.7\n")  # left turn
    s.write("/turn2/run -100 -0.7\n".encode())
    time.sleep(0.9)
elif (face == 'east'):
    print("/turn2/run -100 0.7\n")  # right turn
    s.write("/turn2/run -100 0.7\n".encode())
    time.sleep(0.9)
print("/stop/run \n")
s.write("/stop/run \n".encode())
# run d2
print("/goStraight/run 100 \n")
s.write("/goStraight/run 100 \n".encode())
time.sleep(t2)
print("/stop/run \n")
s.write("/stop/run \n".encode())
print("done\n")
s.write("done\n".encode())


# a = input()
# b = int (input())
# c, d = map(int, input().split())
# print(a, b)
# print(type(a), type(b))
# print(c, d)
# print(type(c), type(d))