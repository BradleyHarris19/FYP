import cv2
import socket
import pickle
import struct

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    client_socket.connect(('10.0.0.10', 5555))
except ConnectionRefusedError as e:
    client_socket.connect(('10.0.0.10', 5565))
    print(f"Error: {e}")
    print("The server does not exist or cannot be reached.")

data = b""
payload_size = struct.calcsize("Q")

# Define video codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('april_video.mp4', fourcc, 10.0, (320, 240))  # Adjust parameters as needed

while True:
    while len(data) < payload_size:
        packet = client_socket.recv(4 * 1024)  # 4K buffer size
        if not packet:
            break
        data += packet

    if not data:
        break

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q", packed_msg_size)[0]

    while len(data) < msg_size:
        data += client_socket.recv(4 * 1024)  # 4K buffer size

    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data)
    cv2.imshow('Client', frame)
    
    # Write frame to video file
    out.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release VideoWriter object and destroy OpenCV windows
out.release()
cv2.destroyAllWindows()
