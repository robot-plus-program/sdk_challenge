from ketisdk.base.udp import UDPClientThread
import numpy as np

client = UDPClientThread()
# client.send({'rgb':np.ones((300, 300)), 'depth': np.zeros((200,200))})
client.send(np.ones((600, 600)))
# client.send({1:2})
