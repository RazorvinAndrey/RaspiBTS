from threading import Thread
import time
import pygame
import psutil
import subprocess
import sys

pygame.init()
screen = pygame.display.set_mode((1080, 720), pygame.RESIZABLE)
pygame.display.update()

# Переменная для приёма данных от пользователя
out = ""


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


# Новая программа от пользователя
def new_proc():
    print("open")

    proc = subprocess.Popen(["python", "test.py"], stdout=subprocess.PIPE)
    while not stop and proc.returncode is None:
        instr = proc.stdout.readline().decode("utf8")
        print(instr)
    try:
        proc.wait(timeout=1)
    except subprocess.TimeoutExpired:
        kill(proc.pid)


stop = False
flag = False
mainloop = True
i = 0
while mainloop:
    keys = pygame.key.get_pressed()
    if keys[pygame.K_2]:
        if not flag:
            print("Process starting...")
            stop = False
            new = Thread(target=new_proc)
            new.start()
            flag = True
    elif keys[pygame.K_3]:
        if flag:
            print("Process stop")
            stop = True
            flag = False
    elif keys[pygame.K_1]:
        print("Change button")
        with open("text.txt", "w") as f2:
            f2.write(str(i))
            f2.close()
    else:
        pass
    with open("state.txt", "w") as f3:
        f3.write(str(i+2) + " " + str(i*2))
        f3.close()
    i = i + 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            mainloop = False
    pygame.display.update()
