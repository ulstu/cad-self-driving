import socket
import traceback

HOST = "192.168.1.188"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.bind((HOST, PORT))
#     s.listen()
#     conn, addr = s.accept()
#     with conn:
#         print(f"Connected by {addr}")
#         while True:
#             data = conn.recv(1024)
#             if not data:
#                 break
#             print(f'received message: {data}')
#             if data.decode('ascii') == 'greenstart':
#                 print('received')
#             #conn.sendall(data)


try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(b"greenstart")
except Exception as err:
    print(f'Socket send error: {traceback.format_exc()}')
