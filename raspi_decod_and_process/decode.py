import base64
import json



def encode(message):
    message_bytes = message.encode('ascii') # Кодируем символы в ASCII формат(преобразуем сообщение в байтоподобный объект)
    base64_bytes = base64.b64encode(message_bytes) # кодируем в base64
    base64_message = base64_bytes.decode('ascii') # расшифровываем закодированную строку в Base64

    return(base64_message)



def decode(base64_message):
    base64_bytes = base64_message.encode('ascii')
    message_bytes = base64.b64decode(base64_bytes)
    message = message_bytes.decode('ascii')

    return (message)


f = open("testo.py", 'r')
s = encode(str(f.read()))
f.close()
print(decode(s))

f2 = open("user.py", "w")
f2.write(decode(s))
f2.close()
'''
f.write(decode(encode())

subprocess.Popen(['python /home/exp1/raspberry_code/testo.py'], shell=True)'''

