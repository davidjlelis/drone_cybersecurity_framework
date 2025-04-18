import socket
import threading
import time
from cryptography.fernet import Fernet
import sys

sys.path.append("..")  # Allow imports from the parent directory
from key_manager import encryption_key  # Import the shared key


HOST = '127.0.0.1'
PORT = 5555
MAX_CONNECTIONS_PER_WINDOW = 10  # Threshold for jamming detection
WINDOW_DURATION = 5  # Time window in seconds

cipher = Fernet(encryption_key)

# Connection tracking
connection_timestamps = []
lock = threading.Lock()
server_running = True  # Flag to control the server loop

def detect_jamming():
    """Detects if too many connections occur in a short time window."""
    global connection_timestamps
    with lock:
        current_time = time.time()
        # Remove timestamps older than WINDOW_DURATION
        connection_timestamps = [t for t in connection_timestamps if current_time - t < WINDOW_DURATION]

        if len(connection_timestamps) >= MAX_CONNECTIONS_PER_WINDOW:
            print("🚨 WARNING: Possible jamming detected! Too many connections in a short time.")
            return True
    return False

def handle_client(conn, addr):
    """Handles communication with a connected client."""
    global connection_timestamps
    with lock:
        connection_timestamps.append(time.time())  # Log new connection

    if detect_jamming():
        print(f"⚠️ Blocking connection from {addr} due to possible jamming.")
        conn.send(cipher.encrypt(b"Too many connections! Possible jamming detected."))
        conn.close()
        return

    print(f"🔗 New connection from {addr}")
    try:
        with open(f"data_from_{addr}.txt", "x") as file:
            file.write(f"File created for {addr}\n")
    except FileExistsError:
        print("File already exists.")
        open(f"data_from_{addr}.txt", "w").close()
    try:
        while True:
            encrypted_data = conn.recv(1024)
            #print(f"Encrypted Data Received from {addr}: {encrypted_data.decode()}")
            
            data = cipher.decrypt(encrypted_data)
            if not data:
                break
            #print(f"Decrypted Data: {data.decode()}")

            # conn.sendall(b"ACK: " + data)
            response = f"ACK: {data}".encode()
            encrypted_response = cipher.encrypt(response)
            conn.send(encrypted_response)

            with open(f"data_from_{addr}.txt", "a") as file:
                file.write(f"Encrypted Data Received: {encrypted_data.decode()}\n")
                file.write(f"Decrypted Data Received: {data.decode()}\n")

    except ConnectionResetError:
        print(f"⚠️ Connection lost with {addr}")
    finally:
        conn.close()
        print(f"🔌 Connection closed: {addr}")

def accept_clients(server_socket):
    """Accepts client connections in a loop."""
    global server_running
    while server_running:
        try:
            server_socket.settimeout(1.0)  # Avoid blocking indefinitely
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
        except socket.timeout:
            continue  # Keep checking if the server is running
        except OSError:
            break  # Server socket closed

# Start server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()
print(f"🚀 Server listening on {HOST}:{PORT}")

# Start accepting clients in a separate thread
accept_thread = threading.Thread(target=accept_clients, args=(server_socket,), daemon=True)
accept_thread.start()

# Main loop for server shutdown
while True:
    command = input("Enter 'x' to exit server: ").strip().lower()
    if command == 'x':
        print("🛑 Shutting down server...")
        server_running = False  # Signal the accept_clients thread to stop
        server_socket.close()  # Close the server socket
        accept_thread.join()  # Wait for thread to exit
        break

