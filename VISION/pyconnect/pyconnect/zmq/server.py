import zmq
from pyconnect.utils import dict2byte, byte2dict, data_info, Timer


class ZmqServer:
    def __init__(self, host='0.0.0.0', port=8888, **kwargs):
        self.host, self.port = host, port
        self.run_func = None
        context = zmq.Context()
        self.sock = context.socket(zmq.REP)
        self.sock.bind(f"tcp://{host}:{port}")

    def listen(self, run_thread=False):
        while True: 
            print('waiting for connection')
            byte_data = self.sock.recv()
            print(f'Byte data received...')
            timer = Timer()
            data = byte2dict(byte_data)
            print(f'{data_info(data)}')
            timer.pin_time('byte2dict')
            if data=='hello':
                ret_data = f'Hi from {self.host}:{self.port}'
            else:
                ret_data = data if self.run_func is None else self.run_func(**data)
            timer.pin_time('process')
            byte_data = dict2byte(ret_data)
            timer.pin_time('dict2byte')
            self.sock.send(byte_data)
            timer.pin_time('send_data')
            print(timer.pin_times_str)

def run_server(custom_func=None):
    from pyconnect.utils import parse_keys_values
    HOST, PORT = '0.0.0.0', 8888
    args = parse_keys_values(optional_args={'host':HOST, 'port':PORT})
    server = ZmqServer(host=args['host'], port=args['port'])

    
    # custom_func = lambda x:x if custom_func is None else custom_func
    if custom_func is None:
        def run_func(**kwargs):
            return kwargs
        server.run_func = run_func
    else:
        server.run_func = custom_func
    server.listen()

if __name__ == '__main__':

    run_server()

