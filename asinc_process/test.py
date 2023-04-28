import time


def GetPosition():
    with open("state.txt", "r") as f:
        string = str(f.read())
        if not (string == ''):
            temp_str = string.split(' ')
            if not (temp_str[0] == ''):
                x = float(temp_str[0])
                y = float(temp_str[1])
            return x, y

i = 0
while True:
    # data = GetPosition()
    # if data:
    #     x = data[0]
    #     y = data[1]
    #     print("x = ", x)
    #     print("y = ", y)
    print(i, flush=True)
    i = i + 1




