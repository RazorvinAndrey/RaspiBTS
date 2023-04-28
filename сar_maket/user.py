import RPi.GPIO as GPIO
import time
import cv2
import sys
#sys.path.insert(1, '/home/Car_maket/i2clibraries')
#from i2c_itg3205 import *
#from i2c_adxl345 import *

# Запуск гироскопа и акселерометра
#itg3205 = i2c_itg3205(1)
#adxl345 = i2c_adxl345(1)


# def GetGiro():
#     (itgready, dataready) = itg3205.getInterruptStatus()
#     if dataready:
#         temp = itg3205.getDieTemperature()
#         (x, y, z) = itg3205.getDegPerSecAxes()
#         print("Temp:" + str(temp))
#         print("X:" + str(x))
#         print("Y:" + str(y))
#         print("Z:" + str(z))
#         print("")
#         return x, y, z
#     time.sleep(1)

#
# def GetAxiler():
#     print (adxl345)
#     time.sleep (1)
#     return adxl345


# Set the type of GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor drive interface definition
ENA = 13  # L298 Enable A
ENB = 20  # L298 Enable B
IN1 = 19  # Motor interface 1
IN2 = 16  # Motor interface 2
IN3 = 21  # Motor interface 3
IN4 = 26  # Motor interface 4

ECHO = 4  # Sensor distance
TRIG = 17  # Sensor distance

IR_R = 18  # Rotate the camera vertically
IR_L = 27  # Rotate the camera horizontally

# Motor initialized to LOW
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)


def ClearPins():
    GPIO.cleanup()
    exit()
    return


def GetImage():
    '''
      Return picture from video camer
   '''
    cam = cv2.VideoCapture(0)
    ret, image = cam.read()
    if ret:
        cv2.imShow(image)
    return image


def GetDistance():
    '''
      Determine the distance to the nearest obstacle in centimeters. Return distance and error.
      If everything is OK: distance, "OK"
      If the response waiting time is exceeded: 0, "Timeout"
      If the distance exceeds 3 meters or is equal to 0: distance, "Out of reach"
   '''

    StartTime = time.time()
    StopTime = time.time()
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        StartTime = time.time()

    while GPIO.input(ECHO) == 1:
        StopTime = time.time()
    pulseDuration = (StopTime - StartTime)
    distance = pulseDuration * 17000;
    if pulseDuration >= 0.01746:
        return 0, 'Time out'
    elif distance > 300 or distance == 0:
        return distance, 'Out of range'
    print("Distance = {} cm.".format(round(distance, 3)))

    return distance, 'Ok'


def MoveCameraVertical(angle):
    '''
      Rotate the camera vertically by a certain angle in the range [60, 120],
      where 60 is the lowest camera position, 120 is the highest.
      The function returns the photo from the position to which the rotation led.
      If the angle is not a number, no rotation will occur.
   '''
    try:
        angle = int(angle)
        if 60 <= angle <= 120:
            pwm = GPIO.PWM(IR_R, 50)
            pwm.start(7.5)
            pwm.ChangeDutyCycle(angle / 18. + 3.)
            time.sleep(0.2)
            pwm.stop()
    except:
        print("Not number")


def MoveCameraHorizontal(angle):
    '''
      Rotate the camera horizontally by a certain angle in the range [35, 145],
      where 60 is the lowest camera position, 120 is the highest.
      The function returns the photo from the position to which the rotation led.
      If the angle is not a number, no rotation will occur.
   '''
    try:
        angle = int(angle)
        if 35 <= angle <= 145:
            pwm = GPIO.PWM(IR_L, 50)
            pwm.start(7.5)
            pwm.ChangeDutyCycle(angle / 18. + 3.)
            time.sleep(0.2)
            pwm.stop()
    except:
        print("Not number")


def MotorForward():
    print('motor forward')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)


def MotorBackward():
    print('motor backward')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)


def MotorTurnRight():
    print('motor turn right')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)


def MotorTurnLeft():
    print('motor turn left')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)


def MotorStop():
    print('motor stop')
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)


def GetPosition():
    with open("state.txt", "r") as f:
        string = str(f.read())
        if not (string == ''):
            temp_str = string.split(' ')
            if not (temp_str[0] == ''):
                x = float(temp_str[0])
                y = float(temp_str[1])
            return x, y


while True:
    MotorForward()
    time.sleep(0.8)
    MotorTurnRight()
    time.sleep(0.4)
    MotorForward()
    time.sleep(2)
    MotorStop()
    break
while True:
    print("program done!", flush=True)
    time.sleep(3)


ClearPins()
