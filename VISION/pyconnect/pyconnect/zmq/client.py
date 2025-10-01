import zmq
from pyconnect.utils import byte2dict, dict2byte, Timer, data_info


class ZmqClient:
    
    def __init__(self, host='localhost', port=8888,**kwargs):
        self.host, self.port = host, port
        context = zmq.Context()
        self.sock = context.socket(zmq.REQ)
        self.sock.connect(f"tcp://{host}:{port}")
        self.send('hello')

    def send(self, data, **kwargs):
        timer = Timer()
        byte_data = dict2byte(data)
        timer.pin_time('dict2byte')
        self.sock.send(byte_data)
        timer.pin_time('send_data')
        byte_data = self.sock.recv()
        timer.pin_time('get_byte')
        rev_data = byte2dict(byte_data)
        print(f'{data_info(rev_data)}')
        timer.pin_time('byte2dict')
        print(timer.pin_times_str)
        return rev_data
        
        

def test_client(data=None):
    from pyconnect.utils import parse_keys_values
    import numpy as np
    # HOST, PORT = '10.252.216.245', 8888
    HOST, PORT = 'localhost', 8888
    args = parse_keys_values(optional_args={'host':HOST, 'port':PORT})
    
    if data is None:
        data = {'text': 'hello', 'rgb': np.random.randint(0,255, size=(2000, 2000,3), dtype='uint8'), 
                 'depth': np.random.randint(0, 65000, size=(2000, 2000), dtype='uint16')}
    
    client = ZmqClient(host=args['host'], port=args['port'])
    client.send(data)

if __name__=='__main__':
    test_client()