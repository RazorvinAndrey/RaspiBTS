import time
import os
import subprocess

j = 0
proc = subprocess.Popen(['python', 'test.py'], creationflags=subprocess.CREATE_NEW_CONSOLE)
while True:
    with open('pid.txt', 'w') as f:
        f.write(str(proc.pid))
    time.sleep(2)
    j = j + 2
    print("Прошло ", j, "секунд")
    if j == 10:
        with open('pid.txt', 'r') as f:
            script_pid = int(f.read())

        os.kill(script_pid, 9)
        break

print("Процесс завершился успешно")
