import socket
import threading

HOST = '127.0.0.1'
PORT = 5555
server_running = True  # Flag to control the server loop

def handle_client(conn, addr):
    """Handles communication with a connected client."""
    print(f"🔗 New connection from {addr}")
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"📡 Received from {addr}: {data.decode()}")
            conn.sendall(b"ACK: " + data)
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

