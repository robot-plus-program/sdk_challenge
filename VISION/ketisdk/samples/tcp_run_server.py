from ketisdk.base.base_tcp import ServerThread

host = 'localhost'
port= 8888

server = ServerThread(host=host, port=8888)
server.listen()
