from cryptography.fernet import Fernet
import socket
import threading
import time
import sys
from my_project.key_manager import load_key  # Load the shared key

KEY = load_key()
cipher = Fernet(KEY)

JAM_THRESHOLD = 3.0  # Response time threshold (seconds)
MAX_CONNECTIONS_PER_SECOND = 10  # Threshold for jamming detection

connection_times = []

def server(host='127.0.0.1', port=5555):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")

    def listen_for_exit():
        """Waits for the user to press 'X' to close the server."""
        while True:
            user_input = input().strip().lower()
            if user_input == "x":
                print("Shutting down server...")
                server_socket.close()
                sys.exit(0)

    threading.Thread(target=listen_for_exit, daemon=True).start()

    while True:
        try:
            client_socket, addr = server_socket.accept()
            
            # Track connection timestamps
            now = time.time()
            connection_times.append(now)
            connection_times[:] = [t for t in connection_times if now - t < 1]  # Keep last 1 sec

            # Check for jamming condition
            if len(connection_times) > MAX_CONNECTIONS_PER_SECOND:
                detect_jam(len(connection_times))
                server_socket.close()
                sys.exit(0)
                

            print(f"Connection from {addr}")
            threading.Thread(target=handle_client, args=(client_socket,)).start()
        except OSError:
            break

def handle_client(client_socket):
    try:
        start_time = time.time()
        
        encrypted_message = client_socket.recv(1024)
        decrypted_message = cipher.decrypt(encrypted_message).decode()
        print(f"Received (Decrypted): {decrypted_message}")

        time.sleep(1)  # Simulate processing delay

        response = f"ACK: {decrypted_message}".encode()
        time_to_send = time.time() - start_time

        if time_to_send > JAM_THRESHOLD:
            detect_jam(time_to_send)

        client_socket.send(cipher.encrypt(response))
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()

def detect_jam(event):
    """Function to handle detection of a jam event."""
    if isinstance(event, float):
        print(f"[WARNING] Response time exceeded: {event:.2f}s (Possible Jamming). Closing server.")
    elif isinstance(event, int):
        print(f"[WARNING] High connection rate detected: {event} connections/sec (Possible Jamming). Closing server.")

if __name__ == "__main__":
    server()

