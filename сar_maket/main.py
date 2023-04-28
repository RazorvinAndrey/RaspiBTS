from threading import Thread
import numpy as np
import time
import psutil
import subprocess
import RPi.GPIO as GPIO
import serial
import websocket
import ssl
import base64
import json
import cv2
from kalman_filter.log_reader import reader_logs
from kalman_filter.Kalman_filter import EKF3


# User login data
login = "logincar2"
password = "Or2f6Q2oTQ"
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

# config video stream
cap = cv2.VideoCapture(0, cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FPS, 12)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

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

# Users params
api = ""
stop = 1

# Server command control
def on_message(ws, mess):
    jmes = json.loads(mess)
    print("on_message ", jmes)
    if jmes["action"] == "CarLogin":
        global api
        api = jmes["data"]["apikey"]
    elif jmes["action"] == "SendCommand":
        if jmes["data"]["command"] == "forward":
            control_key = "forward"
        elif jmes["data"]["command"] == "backward":
            control_key = "down"
        elif jmes["data"]["command"] == "turnLeft":
            control_key = "left"
        elif jmes["data"]["command"] == "turnRight":
            control_key = "right"
        else:
            control_key = "stop"
        RemoteControl(control_key)
    elif jmes["action"] == "SendFile":
        print("take file")
        s = jmes["data"]["bytes"]
        print("create user file")
        with open("user.py", "w") as f:
            f.write(Decode(s))
            f.close()
        print("File get")
    elif jmes["action"] == "RunProgramm":
        global stop
        if jmes["data"]["command"] == "start":
            if stop == 1:
                print("Process starting...")
                new = Thread(target=NewProc)
                new.start()
                stop = 0
                print(stop)
        elif jmes["data"]["command"] == "stop":
            if stop == 0:
                print("Process stop")
                stop = 1
                print(stop)


def video_stream(cap):
    ret, frames = cap.read()
    frames = frames[:, :, 0]
    if ret:
        ret, buffer1 = cv2.imencode('.webp', frames, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        if ret:
            return frames
        else:
            print('Error')
            return 0
    else:
        return 0


def sendPos(ws, x, y):
    j_mes = json.dumps({"action": "SendPos", "data": {"apikey": api, "position": {"x": x, "y": y}}})
    ws.send(j_mes)


def sendLog(ws, x):
    j_mes = json.dumps({"action": "SendLog", "data": {"apikey": api, "log": x}})
    ws.send(j_mes)


def SendVideo(ws, x):
    j_mes = json.dumps({"action": "SendVideo", "data": {"apikey": api, "frame": x}})
    ws.send(j_mes)


def Close(ws):
    j_mes = json.dumps({"action": "Close", "data": {"apikey": api}})
    ws.send(j_mes)


def on_close(ws):
    ws.close()


def on_open(ws):
    print("Connection open")
    j_mes = json.dumps({"action": "CarLogin", "data": {"login": login, "password": password}})
    ws.send(j_mes)


def ClearPins():
    GPIO.cleanup()
    exit()
    return


# Motor control
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
        MotorTurnLeft()
    elif control == "left":
        MotorTurnRight()
    else:
        MotorStop()


def Encode(code):
    message_bytes = code.encode('ascii')  # Кодируем символы в ASCII формат(преобразуем сообщение в байтовый объект)
    base64_bytes = base64.b64encode(message_bytes)  # кодируем в base64
    base64_message = base64_bytes.decode('ascii')  # расшифровываем закодированную строку в Base64

    return base64_message


def Decode(base64_message):
    ase64_bytes = base64_message.encode('ascii')
    message_bytes = base64.b64decode(ase64_bytes)
    code = message_bytes.decode()

    return code


# Trash users program
def Kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


# New program users
def NewProc():
    print("open")
    proc = subprocess.Popen(["python", "user.py"], stdout=subprocess.PIPE, stdin=subprocess.PIPE, text=True)
    global stop
    print(stop)
    while stop == 0 and proc.returncode is None:
        instr = proc.stdout.readline()
        if instr != '':
            print(instr)
            sendLog(ws, instr)
    try:
        proc.wait(timeout=1)
    except subprocess.TimeoutExpired:
        Kill(proc.pid)
    sendLog(ws, "Programm closed")
    print("close")


ws = websocket.WebSocketApp("wss://194.67.86.110:9000", on_message=on_message, on_close=on_close, on_open=on_open)
ssl_options = {"cert_reqs":ssl.CERT_NONE, "ssl_version": ssl.PROTOCOL_TLSv1_2}
new_p = Thread(target=ws.run_forever, kwargs={"sslopt": ssl_options})
new_p.start()
time.sleep(5)

print("api = ", api)

mainloop = True

# Program
with serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = '/dev/ttyACM0'
    ser.open()
    ser.timeout = 1
    ser.write(b'\r\r')
    print(ser.portstr)
    new_str = ser.read_until(b'\n')
    ser.write(b'les\n')
    ser.write(b'\r\r')
    ser.write(b'les\n')
    log = ""
    while mainloop:
        time.sleep(0.001)
        frames = video_stream(cap).flatten().tolist()
        SendVideo(ws, frames)
        if ser.inWaiting():
            new_str = ser.readline()
        if len(new_str) > 10:
            q = new_str.decode('ascii')
            log = str(q)
            print(log)
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
            sendPos(ws, x_est[0] * 100, x_est[1] * 100)
        except:
            pass

print("end")
ClearPins()
cap.release()
cv2.destroyAllWindows()
