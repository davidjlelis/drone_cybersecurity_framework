import socket
import threading

LISTEN_PORT = 5555  # Proxy listens here (same as real server)
FORWARD_HOST = "127.0.0.1"
FORWARD_PORT = 5556  # Redirect traffic to MITM Proxy

def handle_client(client_socket):
    """Handles client connection and forwards traffic to real server."""
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.connect((FORWARD_HOST, FORWARD_PORT))

        # Start two threads to relay traffic in both directions
        threading.Thread(target=relay_traffic, args=(client_socket, server_socket)).start()
        threading.Thread(target=relay_traffic, args=(server_socket, client_socket)).start()

    except Exception as e:
        print(f"Error handling client: {e}")
        client_socket.close()

def relay_traffic(source_socket, destination_socket):
    """Relays data between sockets."""
    try:
        while True:
            data = source_socket.recv(4096)
            if not data:
                break
            print(f"Intercepted Data: {data}")  # Can modify or log data here
            destination_socket.sendall(data)
    except Exception as e:
        print(f"Traffic relay error: {e}")
    finally:
        source_socket.close()
        destination_socket.close()

def start_proxy():
    """Starts the proxy server."""
    proxy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    proxy_socket.bind(("0.0.0.0", LISTEN_PORT))
    proxy_socket.listen(5)
    print(f"Transparent proxy listening on port {LISTEN_PORT}...")

    while True:
        client_socket, addr = proxy_socket.accept()
        print(f"New connection from {addr}")
        threading.Thread(target=handle_client, args=(client_socket,)).start()

if __name__ == "__main__":
    start_proxy()

