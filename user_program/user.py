import RPi.GPIO as GPIO

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


while True:
    # Чтение координат с приёмника СШП
    with open("state.txt", "r") as f:
        string = str(f.read())
        temp_str = string.split(' ')
        x = float(temp_str[0])
        y = float(temp_str[1])

    print("x = ", x)
    print("y = ", y)


ClearPins()
