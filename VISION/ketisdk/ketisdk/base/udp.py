from ketisdk.base.utils import show_data, byte2ins, ins2byte, recvall
from threading import Thread
import socket
from ketisdk.base.utils import decorate_show_log, decorate_all_class_method

DATA_MAX_LEN = 65000
@decorate_all_class_method(decorate_show_log)
class UDPServerThread():
    def __init__(self, host='localhost', port=8888,max_len=DATA_MAX_LEN):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # , socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.max_len = max_len
        print(f'UDP Server {host} at PORT {port} running ....')

    def listen(self):
        print('{} listening a connection'.format('+' * 10))
        # self.thread = Thread(target=self.listenToClient, daemon=True)
        # self.thread.start()
        self.listenToClient()

    def listenToClient(self):
        while True:
            byteData = b""
            # n_chunks = 0
            while True:
                # n_chunks += 1
                byteChunk, address = self.sock.recvfrom(self.max_len)
                byteData += byteChunk
                if byteData.endswith(b"$$"):
                    byteData = byteData[:-2]
                    break
            # print(f'{n_chunks} chunks received ...')
            self.data = byte2ins(byteData=byteData)
            print(f'{"*" * 10}  Data received from {address}')
            show_data(self.data)


    def terminate(self):
        # self.thread.join()
        self.sock.close()

@decorate_all_class_method(decorate_show_log)
class UDPClientThread():
    def __init__(self, host='localhost', port=8888, max_len=DATA_MAX_LEN):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ,socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host, self.port, self.max_len = host, port, max_len
        print(f'Client connecting to {host} at PORT {port} ...')

    def send(self, data):
        """ send a dict to sever"""
        # while True:
        # try:
        byteData = ins2byte(data)

        # Split to chunks and send
        byteData += b"$$"
        byteLen = len(byteData)
        n_chunks = byteLen//self.max_len
        for i in range(n_chunks):
            self.sock.sendto(byteData[i*self.max_len: (i+1)*self.max_len], (self.host, self.port))
        if n_chunks*self.max_len != byteLen:
            self.sock.sendto(byteData[n_chunks * self.max_len:], (self.host, self.port))
            n_chunks +=1
        # print(f'{n_chunks} chunks sent ....')

        # self.sock.sendto(byteData, (self.host, self.port))
        # except:
        #     self.sock.close()
        #     print('socket closed ...')


def demo_udp_server_client_simple():
    import numpy as np
    import time
    server = UDPServerThread()
    Thread(target=server.listen, daemon=True).start()


    client = UDPClientThread()
    # client.send({'rgb': np.ones((2000, 2000)), 'depth': np.zeros((2000, 2000))})
    client.send(np.ones((10000,10000)))

    time.sleep(2)






if __name__ == "__main__":
    demo_udp_server_client_simple()
