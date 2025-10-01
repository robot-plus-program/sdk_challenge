import socket
import threading
import time
import copy
from pyconnect.utils import byte2dict, dict2byte, data_info, Timer, recvall
from pyinterfaces.utils import vision_mode
class TcpIpServer:
    def __init__(self, host='localhost', port=8888, run_func=None,run_ang_func=None, num_connect=5):
        self.host = host
        self.port = port
        self.run_func = run_func
        self.run_ang_func = run_ang_func
        self.run_func_tisCam=None
        self.run_func_rs=None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(num_connect)  # Allow up to 5 pending connections
        self.active = True
        print(f"Server running on {self.host}:{self.port}")

    def listen(self):
        """Start listening for client connections."""
        print("Waiting for connections...")
        try:
            while self.active:
                client, address = self.sock.accept()
                print(f"Connection established with {address}")
                client.settimeout(1200)  # Timeout after 20 minutes
                threading.Thread(target=self.handle_client, args=(client, address), daemon=True).start()
        except KeyboardInterrupt:
            print("Server shutting down...")
            self.stop()

    def handle_client(self, client, address):
        """Handle communication with a connected client."""
        try:
            while True:
                print(f"Waiting for data from {address}")

                # Receive message length
                length = recvall(client, 16)
                if not length:
                    print(f"Client {address} disconnected")
                    break

                # Receive the actual message
                byte_data = recvall(client, int(length))
                if not byte_data:
                    print(f"No data received from {address}, closing connection")
                    break
                timer = Timer()
                print(f"Data received from {address}")
                received_data = byte2dict(byte_data)
                print(data_info(received_data))
                timer.pin_time('byte2dict')

                # Process the data and prepare a response
                response_data = self.process_data(received_data)
                timer.pin_time('process_data')
                self.send_data(client, response_data)
                timer.pin_time('send_data')

                print(f"Response sent to {address}")
                print(timer.pin_times_str)
        except Exception as e:
            print(f"Error with client {address}: {e}")
        finally:
            client.close()

    def process_data(self, received_data):
        """Process the received data and return the response."""
        if received_data=='hello':
            return f'Hi from server {self.host}:{self.port}'
        else:
            if received_data['mode']==vision_mode.MODE_GRASP:
                print("detect_mode.MODE_GRASP")
                if self.run_func:
                    return self.run_func(**received_data)
            if received_data['mode']==vision_mode.MODE_ANGLE:                
                param=copy.deepcopy(received_data)
                del param['mode']
                ret=self.run_ang_func(**param)
                return [ret,ret['ret_img']]
            if received_data['mode']==vision_mode.MODE_RS:
                if self.run_func_rs==None:
                    return [None,None]
                return self.run_func_rs()
            if received_data['mode']==vision_mode.MODE_TIS:
                if self.run_func_tisCam==None:
                    return [None,None]
                return self.run_func_tisCam()
        return received_data  # Echo back the data if no processing function is provided

    def send_data(self, client, data):
        """Send data to the client."""
        byte_data = dict2byte(data)
        length = str(len(byte_data)).rjust(16, '0').encode()
        client.sendall(length + byte_data)

    def stop(self):
        """Stop the server and close the socket."""
        self.active = False
        self.sock.close()
        print("Server stopped.")

def run_server(custom_func=None):
    from pyconnect.utils import parse_keys_values
    HOST, PORT = '0.0.0.0', 8888
    args = parse_keys_values(optional_args={'host':HOST, 'port':PORT})
    
    custom_func = lambda x:x if custom_func is None else custom_func
    server = TcpIpServer(host=args['host'], port=args['port'])
    server.run_func = custom_func
    server.listen()
    
    

if __name__ == '__main__':

    run_server()
