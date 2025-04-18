import socket
import threading
import sys

# MITM Proxy Configuration
LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 5555  # Fake server (hacker)
REAL_SERVER_HOST = "127.0.0.1"
REAL_SERVER_PORT = 5556  # Real server

stop_event = threading.Event()  # Event to signal shutdown

def intercept_data():
    hacker_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    hacker_socket.bind((LISTEN_HOST, LISTEN_PORT))
    hacker_socket.listen(5)
    print(f"MITM Proxy listening on {LISTEN_HOST}:{LISTEN_PORT}")

    # Start a separate thread to listen for exit input
    threading.Thread(target=listen_for_exit, args=(hacker_socket,), daemon=True).start()

    try:
        with open(f"MitM_data_intercept.txt", "x") as file:
            file.write("File created for Man-in-the-Middle data interception.\n")
    except FileExistsError:
        print("File already exists.")
        open("MitM_data_intercept.txt", "w").close()

    while not stop_event.is_set():
        try:
            client_socket, addr = hacker_socket.accept()
            print(f"Intercepted connection from {addr}")

            # Forward to real server
            real_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            real_server_socket.connect((REAL_SERVER_HOST, REAL_SERVER_PORT))

            threading.Thread(target=relay_traffic, args=(client_socket, real_server_socket)).start()
            threading.Thread(target=relay_traffic, args=(real_server_socket, client_socket)).start()
        except OSError:
            break  # Exit the loop when socket is closed

    print("Shutting down MITM proxy...")

def listen_for_exit(hacker_socket):
    """Waits for the user to press 'X' to close the proxy."""
    while True:
        user_input = input().strip().lower()
        if user_input == "x":
            print("Stopping MITM proxy...")
            stop_event.set()
            hacker_socket.close()  # Close the listening socket
            sys.exit(0)

def relay_traffic(source_socket, destination_socket):
    """Relays traffic between two sockets."""
    try:
        while True:
            data = source_socket.recv(1024)
            if not data:
                break
            #print(f"Intercepted Data: {data}")  # Print intercepted message
            with open(f"MitM_data_intercept.txt", "a") as file:
                file.write(f"Intercepted Data: {data.decode()}\n\n") 
            destination_socket.sendall(data)  # Use sendall() for reliability
    except Exception as e:
        print(f"Error in relaying: {e}")
    finally:
        source_socket.close()
        destination_socket.close()


if __name__ == "__main__":
    intercept_data()