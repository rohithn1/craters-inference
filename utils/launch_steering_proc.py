import socket
from motor_control.steering.steering_module import SteeringController

steering_obj = SteeringController(init_sleep_factor=4, pwm_pin=32)

def start_server(host='127.0.0.1', port=65433):
    # Create a TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the address and port
    server_address = (host, port)
    print(f'Starting server on {host}:{port}')
    server_socket.bind(server_address)

    # Listen for incoming connections
    server_socket.listen(5)
    print('Waiting for a connection...')

    try:
        while True:
            # Accept a connection
            connection, client_address = server_socket.accept()
            print(f'Connection from {client_address}')

            try:
                while True:
                    # Receive data from the client
                    data = connection.recv(1024)
                    if data:
                        #print(f'Received: {data.decode()} ({type(data.decode())})')
                        angle = None
                        if '\n' in data.decode():
                            angle = float(data.decode().split('\n')[0])
                        else:
                            angle = float(data.decode())
                        steering_obj._steer(new_direction=angle)        
                    else:
                        # No more data from the client
                        print('No more data. Closing connection.')
                        break

            finally:
                # Clean up the connection
                connection.close()

    except KeyboardInterrupt:
        print('Server shutting down.')
    finally:
        server_socket.close()

if __name__ == '__main__':
    start_server()
