import socket

class TeleopClient():
    UDP_ADDRESS = '192.168.2.58'
    UDP_PORT = 8484
    MESSAGE = "%04d%04d"

    def __init__(self):
        print("init")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def send(self, left, right):
        print("send")
        self.sock.sendto(str.encode(TeleopClient.MESSAGE%(left,right)), (TeleopClient.UDP_ADDRESS, TeleopClient.UDP_PORT))

    def start(self):
        print("start")
        self.send(50,0)

if __name__=='__main__':
    tc = TeleopClient()
    tc.start()