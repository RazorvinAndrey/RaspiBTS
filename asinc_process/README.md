---
# Тестирование работы параллельного процесса

- в программе test.py находиться алгоритм чтения из файла данных по координатам и выдача их обратно в консоль, для связи между двумя параллельными программами.
- в программе test_new_proc.py находится основная программа, в которой запускается параллельный процесс и обрабатываются данные из него.

Для запуска параллельного процесса используется библиотека subprocess, вот алгоритм для запуска и остановки параллельного процесса:
    
    proc = subprocess.Popen(["python", "test.py"], stdout=subprocess.PIPE, stdin=subprocess.PIPE, text=True)
    while not stop and proc.returncode is None:
        instr = proc.stdout.readline()
        print(instr)
        time.sleep(1)
    try:
        proc.wait(timeout=1)
    except subprocess.TimeoutExpired:
        kill(proc.pid)

    def kill(proc_pid):
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

Тут в файле test.py находиться исполняемая программа и она запускается с помощью консольной команды "python test.py".

Вывод из параллельной консоли происходит в переменную "instr". 

Для завершения процесса нужно прервать цикл чтения строки из параллельной консоли и вызвать функцию "kill", которая уничтожит выполнение параллельного процесса.
