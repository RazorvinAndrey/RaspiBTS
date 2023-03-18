import RPi.GPIO as GPIO
import time
from mdechello import make_anchor
import serial
from log_reader import reader_logs
import numpy as np
from Kalman_filter import EKF3
import serial

def ClearPins():
    GPIO.cleanup()
    exit()
    return

def MotorForward():
    '''
      The car is going forward.
      The movement will continue until another motion function or the MotorStop() function is called.
   '''
    print('motor forward')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def MotorBackward():
    '''
      The car is going forward.
       The movement will continue until another motion function or the MotorStop() function is called.
   '''
    print('motor backward')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)

def MotorTurnRight():
    '''
      The car turns right. The turn will continue until another motion function or the MotorStop() function is called.
   '''
    print('motor turnright')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)

def MotorTurnLeft():
    '''
      The car turns left. The turn will continue until another motion function or the MotorStop() function is called.
   '''
    print('motor turnleft')
    GPIO.output(ENA, True)
    GPIO.output(ENB, True)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

def MotorStop():
    '''
      The car stops. The car will continue to stand until another movement is called.
   '''
    print('motor stop')
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)

# Set the type of GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

######## Motor drive interface definition ##############################################################################
ENA = 13  # //L298 Enable A
ENB = 20  # //L298 Enable B
IN1 = 19  # //Motor interface 1
IN2 = 16  # //Motor interface 2
IN3 = 21  # //Motor interface 3
IN4 = 26  # //Motor interface 4


######### Motor initialized to LOW #####################################################################################
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)



# Константы
X_prev = np.array([0.6, 1.5, 0.3, 0.03, 0, 0, 0, 0, 0])  # начальное положение
D_n_UWB = 0.0009
T_sample = 0.1
x_sat = np.zeros((4, 3))
D_n_mat = D_n_UWB * np.eye(4)
x_est_prev = X_prev
D_x_prev = np.eye(X_prev.shape[0])

list_x = []
list_y = []
z="ZZZZZZZZZZZZZZZZZZZZzz"
with serial.Serial() as ser:  # содержимое порта сохраняется в переменную ser, затем используется в дальнейшем
    ser.baudrate = 115200  # скорость передачи данных
    ser.port = '/dev/ttyACM0'  # выбор порта передачи
    ser.open()  # открываем файл, чтобы достать содержимое
    ser.write(b'\r\r')  # Для передачи данных используется метод write. Ему нужно передавать байтовую строку
    
    print(ser.portstr)
    new_str = ser.read_until(b'\n')
    ser.write(b'les\n')
    log = ""
    step = 0
    while step < 1000:
        
        new_str = ser.read_until()  # Перебор с указанием последней строки
        if len(new_str) > 10:
            q = new_str.decode('ascii')
            log = str(q)
            print(log)
        if new_str == b' Help      :  ? or help\r\n':  # когда дошел до последней строки печатает ее
            last_str = ser.read_until(b'\n')
            print(last_str)
            break

      #  store = reader_logs(log)
     #   list_keys = []
    #    j = 0
   #     print(store)
        #try:
         #   for key in store:
        #        list_keys.append(key)
       #         x_sat[j] = store[key]['x_sat']
            #    j = j + 1
           # R = {}
          #  for key in store:
         #       R[key] = store[key]['range']
       #     print(R)
        #    x_est, D_x = EKF3(R, x_est_prev, D_x_prev, D_n_mat, x_sat, T_sample, list_keys)
       #     D_x_prev = D_x
       #     x_est_prev = x_est
            #list_x.append(x_est[0])
         #   list_y.append(x_est[1])
      #  except:
          #  pass
        #MotorForward()
        step = step+1
        
print("Stop")
MotorStop()
time.sleep(1)
make_anchor(list_x[len(list_x)], list_y[len(list_y)])
