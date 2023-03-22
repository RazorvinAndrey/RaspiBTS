from threading import Thread
import numpy as np
import time
import base64
import psutil
import subprocess
import RPi.GPIO as GPIO
import pygame
import serial
import websocket
import json
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

# Переменная для приёма данных от пользователя
out = ""

api = ''


# Обработчики для подключения к серверу
def on_message(ws_l):
    return json.loads(ws_l.recv())


def on_error(ws_l):
    x = json.loads(ws_l.recv())
    if x["status"] == "false":
        print("Error")


def sendPos(ws_l, x, y):
    j_mes = json.dumps({"action": "SendPos", "data": {"apikey": api, "position": {"x": x, "y": y}}})
    ws_l.send(j_mes)


def sendLog(ws_l, x):
    j_mes = json.dumps({"action": "SendLog", "data": {"log": "x"}})
    ws_l.send(j_mes)


def on_close(ws_l):
    x = json.loads(ws_l.recv())
    if x["action"] == "Close":
        ws_l.close()


def on_open(ws_l):
    print("Connection open")
    j_mes = json.dumps({"action": "carLogin", "data": {"login": "***", "password": "***"}})
    ws_l.send(j_mes)
    req = ws_l.recv(timeout=100)
    if message:
        print("Connection failes")
    else:
        ans = json.loads(req)
        if ans["status"] == "true":
            print("Connection comleted")
            api = ans["data"]["apikey"]
        else:
            print("Connection failed")


def start_websocket(ws_l):
    ws_l.run_forever()  # запуск бесконечного соединения


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
def Encode(code):
    message_bytes = code.encode('ascii')  # Кодируем символы в ASCII формат(преобразуем сообщение в байтовый объект)
    base64_bytes = base64.b64encode(message_bytes)  # кодируем в base64
    base64_message = base64_bytes.decode('ascii')  # расшифровываем закодированную строку в Base64

    return base64_message


# Декодирование
def Decode(base64_message):
    base64_bytes = base64_message.encode('ascii')
    message_bytes = base64.b64decode(base64_bytes)
    code = message_bytes.decode('ascii')

    return code


# Уничтожение дочерней программы
def Kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


# Новая программа от пользователя
def NewProc(ws_l):
    print("open")
    proc = subprocess.Popen(["python", "user.py"], stdout=subprocess.PIPE)
    while not stop and proc.returncode is None:
        instr = proc.stdout.readline().decode("utf8")
        sendLog(ws_l, instr)
        print(instr)
    rc = proc.poll()
    print(rc)
    try:
        proc.wait(timeout=1)
    except subprocess.TimeoutExpired:
        Kill(proc.pid)


ws = websocket.WebSocketApp("***", on_message=on_message, on_error=on_error, on_close=on_close, on_open=on_open)
websocket_thread = Thread(target=start_websocket)
websocket_thread.start()

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
        message = on_message(ws)
        if message["action"] == "SendCommand":
            if message["data"]["command"] == "forward":
                control_key = "forward"
            elif message["data"]["command"] == "backward":
                control_key = "down"
            elif message["data"]["command"] == "turnLeft":
                control_key = "left"
            elif message["data"]["command"] == "turnRight":
                control_key = "right"
            else:
                control_key = "stop"
            RemoteControl(control_key)
        if message["action"] == "sendFirmware":
            if message["data"]["command"] == "create":
                if not file_create:
                    print("create user file")
                    # Тут будет приём строки закодированного файла пользователя
                    s = message["data"]["data_1"]
                    f2 = open("user.py", "w")
                    f2.write(Decode(s))
                    f2.close()
                    file_create = True
            elif message["data"]["command"] == "start":
                if not flag:
                    print("Process starting...")
                    stop = False
                    new = Thread(target=NewProc(ws))
                    new.start()
                    flag = True
            elif message["data"]["command"] == "stop":
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
            sendPos(ws, x_est[0], x_est[1])
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
