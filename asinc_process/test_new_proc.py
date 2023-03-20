from threading import Thread
import time
import pygame
import psutil
import subprocess

pygame.init()
screen = pygame.display.set_mode((1080, 720), pygame.RESIZABLE)
pygame.display.update()


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


# Новая программа от пользователя
def new_proc():
    print("open")
    proc = subprocess.Popen("python test.py", shell=True)
    while not stop:
        print("work")
        time.sleep(1)
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
        f2 = open("text.txt", "w")
        f2.write(str(i))
        f2.close()
    else:
        pass
    i = i + 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            mainloop = False
    pygame.display.update()
