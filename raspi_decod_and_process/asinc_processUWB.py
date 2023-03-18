from threading import Thread
import time
import psutil
import subprocess

stop = True
def main_proc():
    while True:
        time.sleep(1)
        print("proc1")
        

def kill(proc_pid):

    process = psutil.Process(proc_pid)

    for proc in process.children(recursive=True):

        proc.kill()

    process.kill()



def new_proc():
    #subprocess.run(['python', '/home/exp1/raspberry_code/testo.py &'], shell=True)
    '''with open('/home/exp1/raspberry_code/testo.py') as f:
        exec(f.read())'''
        
    '''result = subprocess.Popen(['python /home/exp1/raspberry_code/testo.py'], shell=True)
    while stop:
        with open("pid.txt", "w") as fal:
            fal.write(str(result.pid))
        time.sleep(2)
        print("proc2")
        with open('/home/exp1/raspberry_code/testo.py') as f:
            exec(f.read())
    with open("pid.txt", "r") as fil:
        sript = int(fil.read())
        
    os.kill(sript, 9)
    #result.terminate()'''
    #res = subprocess.Popen( ["lxterminal", "-e", "python", "/home/exp1/raspberry_code/testo.py" ], shell=True)
    #proc = subprocess.Popen("lxterminal -t sample -e bash -c 'python testo.py ; read v '", shell=True)
    proc = subprocess.Popen("python testo.py", shell=True)
    while stop:
        time.sleep(2)
        #with open("pid.txt", "w") as fal:
            #fal.write(str(res.pid))
        
        print("proc2")
    #with open("pid.txt", "r") as fil:
        #sript = int(fil.read())
        #print(sript)
    #p = psutil.Process(sript)
    #os.killpg(os.getpgid(res.pid), signal.SIGTERM)
    #subprocess.check_call("TASKKILL /F /PID {pid} /T".format(pid=res.pid))
    #subprocess.Popen.kill(res)
    '''par = psutil.Process(res.pid)
    chil = par.children(recursive=True)
    for ch in chil:
        ch.kill()
    par.kill()'''
    
    try:

        proc.wait(timeout=3)

    except subprocess.TimeoutExpired:

        kill(proc.pid)
    
    #subprocess.Popen.kill(int(res.pid))


# start the producers
main = Thread(target=main_proc)

#producers = [main, new]
# start the producers
'''for producer in producers:
    producer.start()
# wait for all threads to finish
for producer in producers:
    producer.run()'''

main.start()
time.sleep(5)

new = Thread(target=new_proc)

new.start()
time.sleep(5)
stop = False
