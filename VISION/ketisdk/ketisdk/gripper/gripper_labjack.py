import u3
import time

class hU3_labjack():
    def __init__(self):
        self.d = u3.U3()
        print('labjack :: init')

    def __del__(self):
        self.close()

    def close(self):
        self.d.close()

    def IoOn(self, ioNum = 4):
        print('labjack :: suction IO on')
        self.d.setFIOState(ioNum, state=1)

    def IoOff(self, ioNum = 4):
        print('labjack :: suction IO off')
        self.d.setFIOState(ioNum, state=0)

if __name__ == "__main__":
    suction = hU3_labjack()

    while True:
        suction.IoOff()
        time.sleep(2)

        suction.IoOn()
        time.sleep(2)
