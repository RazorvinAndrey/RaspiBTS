import serial  # необходимая библиотека
import keyboard
import sys
import pygame
pygame.init()
screen = pygame.display.set_mode((240, 120), pygame.RESIZABLE)
pygame.display.update()
file1 = open("log1.txt", 'w')

#0  1000  6576     1  20   0   4460   768 wait   S    pts/7      0:00 /bin/sh -c /usr/bin/python ./log_read2.py.py
#0  1000  6577  6576  20   0  30928  7524 poll_s S    pts/7      0:00 /usr/bin/python


#0  1000  6631     1  20   0   4460   788 wait   S    pts/7      0:00 /bin/sh -c ./log_read2.py.py
#0  1000  6632  6631  20   0  24252  7144 poll_s S    pts/7      0:00 python ./log_read2.py.py

with serial.Serial() as ser:  # содержимое порта сохраняется в переменную ser, затем используется в дальнейшем
    ser.baudrate = 115200  # скорость передачи данных
    ser.port = '/dev/ttyACM0'  # выбор порта передачи
    ser.open()  # открываем файл, чтобы достать содержимое
    ser.timeout = 1
    ser.write(b'\r\r')  # Для передачи данных используется метод write. Ему нужно передавать байтовую строку
    print(ser.portstr)
    new_str = ser.read_until(b'\n')
    ser.write(b'les\n')
    log = ""
    while True:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_2]:
            #sys.exit(0)
            break
        else:
            if ser.inWaiting():
                new_str = ser.readline()
            #try:
                #new_str = ser.readline()  # Перебор с указанием последней строки
            #except:
                #pass
        if len(new_str) > 10:
            q = new_str.decode('ascii')
            log = str(q)
            print(log)
            file1.write(log)
        if new_str == b' Help      :  ? or help\r\n':  # когда дошел до последней строки печатает ее
            last_str = ser.read_until(b'\n')
            print(last_str)
            break
        
        
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                mainloop = False
            
file1.close()
