from cryptography.fernet import Fernet
import socket
import time
from my_project.key_manager import load_key  # Load the shared key

KEY = load_key()
cipher = Fernet(KEY)

def detect_intrusion(start_time):
    """Checks for high latency or dropped connection (indicating a jam)."""
    latency = time.time() - start_time
    if latency > 3:
        print("[ALERT] Jam detected! Exiting program.")
        exit()

def client(message, host='127.0.0.1', port=5555):
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(3)  # Set timeout for detection
        client_socket.connect((host, port))
        
        encrypted_message = cipher.encrypt(message.encode())
        start_time = time.time()
        client_socket.send(encrypted_message)
        
        try:
            response = client_socket.recv(1024)
            if not response:
                raise socket.timeout  # Treat empty response as timeout
            detect_intrusion(start_time)
            print(f"Server Response (Decrypted): {cipher.decrypt(response).decode()}")
        except (socket.timeout, ConnectionResetError):
            print("[ALERT] Jam detected due to timeout/disconnect!")
            exit()
        
        client_socket.close()
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        exit()

if __name__ == "__main__":
    client("Hello, Drone!", host="127.0.0.1", port=5555)

