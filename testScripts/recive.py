import socket

# Replace these with your server details
HOST = '0.0.0.0'
PORT = 8181

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        data = s.recv(1024)
        if not data:
            break
        print(data.decode('utf-8'))
