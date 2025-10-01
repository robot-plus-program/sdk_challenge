import socket, numpy as np
from pyconnect.utils import byte2dict, dict2byte, data_info, Timer, recvall

class TcpIpClient():
    def __init__(self, host='localhost', port=8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = host
        self.port = port
        self.server_connected = False
        self.rev_data = None
        print(f'Client connecting to {host} at PORT {port} ...')
        self.send('hello')

    def send(self, data=None):
        """ send a dict to sever"""
        while True:
            
            try:
                if not self.server_connected:
                    print(f'{"="*10} Connecting to server: {self.host}')
                    self.sock.connect((self.host, self.port))
                    print('Connected ....')
                    self.server_connected = True

                timer = Timer()
                self.send_dict(aDict=data)
                timer.pin_time('send_data')
                ret = self.get_return()
                timer.pin_time('get_return')
                print(timer.pin_times_str)
                return ret
            except:
                self.sock.close()
                print('socket closed ...')
                return None
            
            

    def send_dict(self, aDict):
        print(f'{"="*10} Sending byte data')
        byteData = dict2byte(aDict)
        len_byte = str(len(byteData)).rjust(16, '0').encode()
        self.sock.send(len_byte + byteData)
        print('Byte data sent...')

    def get_return(self):
        print(f'{"="*10} Waiting for return')
        ret_len = recvall(self.sock, 16)
        byteData = recvall(self.sock, int(ret_len))
        print('Return received ...')
        self.rev_data = byte2dict(byteData)
        print(data_info(self.rev_data))
        return self.rev_data

def test_client():
    from pyconnect.utils import parse_keys_values
    HOST, PORT = 'localhost', 8801
    args = parse_keys_values(optional_args={'host':HOST, 'port':PORT})
    
    client = TcpIpClient(host=args['host'], port=args['port'])
    client.send({'text': 'hello', 'rgb': np.random.randint(0,255, size=(2000, 2000,3), dtype='uint8'), 
                 'depth': np.random.randint(0, 65000, size=(2000, 2000), dtype='uint16')})
    
if __name__=='__main__':
    test_client()
    