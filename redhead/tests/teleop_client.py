import socket

class TeleopClient():
    UDP_ADDRESS = '192.168.2.58'
    UDP_PORT = 8484
    MESSAGE = "%04d%04d"
    THROTTLE = .75
    KEY_MAP = {
        'q': (50,100),
        'w': (100,100),
        'e': (100,50),
        'a': (-100,100),
        's': (0,0),
        'd': (100,-100),
        'z': (-50,-100),
        'x': (-100,-100),
        'c': (-100,-50)
    }

    def __init__(self):
        print("init")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def send(self, left, right):
        print("send")
        self.sock.sendto(str.encode(TeleopClient.MESSAGE%(left,right)), (TeleopClient.UDP_ADDRESS, TeleopClient.UDP_PORT))

    def start(self):
        print("start")
        k = None
        while True:
            k = input("stop or key...\nqwe\nasd\nzxc\n? ")
            if k == "stop":
                self.send(0, 0)
                break
            self.send(TeleopClient.KEY_MAP[k[0]][0]*TeleopClient.THROTTLE, TeleopClient.KEY_MAP[k[0]][1]*TeleopClient.THROTTLE)
        print("stop")

if __name__=='__main__':
    tc = TeleopClient()
    tc.start()