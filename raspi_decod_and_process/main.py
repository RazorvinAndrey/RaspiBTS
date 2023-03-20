from threading import Thread
import numpy as np
import time
import base64
import psutil
import subprocess
import RPi.GPIO as GPIO
import pygame
import serial
from raspi_decod_and_process.kalman_filter.log_reader import reader_logs
from raspi_decod_and_process.kalman_filter.Kalman_filter import EKF3

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

# Init pygame
pygame.init()
screen = pygame.display.set_mode((240, 120), pygame.RESIZABLE)
pygame.display.update()

# Константы для фильтра Калмана
X_prev = np.array([0, 0, 0, 0.2, 0, 0, 0, 0, 0])  # начальное положение
D_n_UWB = 0.0009
T_sample = 0.1
x_sat = np.zeros((4, 3))
D_n_mat = D_n_UWB * np.eye(4)
x_est_prev = X_prev
D_x_prev = np.eye(X_prev.shape[0])

list_x = []
list_y = []


def ClearPins():
    GPIO.cleanup()
    exit()
    return


# Управление моторами
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


def RemoteControl(control):
    if control == "forward":
        MotorForward()
    elif control == "down":
        MotorBackward()
    elif control == "right":
        MotorTurnRight()
    elif control == "left":
        MotorTurnLeft()
    else:
        MotorStop()


# Кодирование
def Encode(message):
    message_bytes = message.encode('ascii')  # Кодируем символы в ASCII формат(преобразуем сообщение в байтовый объект)
    base64_bytes = base64.b64encode(message_bytes)  # кодируем в base64
    base64_message = base64_bytes.decode('ascii')  # расшифровываем закодированную строку в Base64

    return base64_message


# Декодирование
def Decode(base64_message):
    base64_bytes = base64_message.encode('ascii')
    message_bytes = base64.b64decode(base64_bytes)
    message = message_bytes.decode('ascii')

    return message


# Уничтожение дочерней программы
def Kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


# Новая программа от пользователя
def NewProc():
    print("open")
    proc = subprocess.Popen("python user.py", shell=True)
    while not stop:
        print("work")
        time.sleep(1)
    try:
        proc.wait(timeout=1)
    except subprocess.TimeoutExpired:
        Kill(proc.pid)


# Флаги для работы переключателей
stop = False
flag = False
file_create = False
mainloop = True
active = "remote_control"
# Основная программа
with serial.Serial() as ser:  # содержимое порта сохраняется в переменную ser, затем используется в дальнейшем
    ser.baudrate = 115200  # скорость передачи данных
    ser.port = '/dev/ttyACM0'  # выбор порта передачи
    ser.open()  # открываем файл, чтобы достать содержимое
    ser.timeout = 1
    ser.write(b'\r\r')  # Для передачи данных используется метод write. Ему нужно передавать байтовую строку
    print(ser.portstr)
    new_str = ser.read_until(b'\n')
    ser.write(b'les\n')
    time.sleep(2)
    ser.write(b'\r\r')
    ser.write(b'les\n')
    log = ""
    while mainloop:
        keys = pygame.key.get_pressed()
        if active == "remote_control":
            if keys[pygame.K_UP]:
                control_key = "forward"
            elif keys[pygame.K_DOWN]:
                control_key = "down"
            elif keys[pygame.K_LEFT]:
                control_key = "left"
            elif keys[pygame.K_RIGHT]:
                control_key = "right"
            else:
                control_key = "stop"
            RemoteControl(control_key)
        if keys[pygame.K_1]:
            if not file_create:
                print("create user file")
                f = open("testo.py", 'r')
                # Тут будет приём строки закодированного файла пользователя
                s = Encode(str(f.read()))
                f.close()
                f2 = open("user.py", "w")
                f2.write(Decode(s))
                f2.close()
                file_create = True
        elif keys[pygame.K_2]:
            if not flag:
                print("Process starting...")
                stop = False
                new = Thread(target=NewProc)
                new.start()
                flag = True
        elif keys[pygame.K_3]:
            if flag:
                print("Process stop")
                stop = True
                flag = False

        # Работа СШП
        if ser.inWaiting():
            new_str = ser.readline()
        if len(new_str) > 10:
            q = new_str.decode('ascii')
            log = str(q)

        # Определение координат
        store = reader_logs(log)
        list_keys = []
        j = 0
        try:
            for key in store:
                list_keys.append(key)
                x_sat[j] = store[key]['x_sat']
                j = j + 1
            R = {}
            for key in store:
                R[key] = store[key]['range']
            x_est, D_x = EKF3(R, x_est_prev, D_x_prev, D_n_mat, x_sat, T_sample, list_keys)
            D_x_prev = D_x
            x_est_prev = x_est
            with open("state.txt", "w") as f3:
                f3.write(str(x_est[0]) + " " + str(x_est[1]))
                f3.close()
            list_x.append(x_est[0])
            list_y.append(x_est[1])
            print("x = ", x_est[0])
            print("y = ", x_est[1])
        except:
            pass

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                mainloop = False
        pygame.display.update()

ClearPins()
print("X ")
print(list_x)
print("Y ")
print(list_y)
