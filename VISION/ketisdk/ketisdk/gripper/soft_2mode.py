import subprocess
import serial
class Soft2PModeGripper:
    def __init__(self, sudo_password='keti1234'):
        command = 'chmod 666 /dev/ttyUSB0'
        command = command.split()
        cmd1 = subprocess.Popen(['echo', sudo_password], stdout=subprocess.PIPE)
        cmd2 = subprocess.Popen(['sudo', '-S'] + command, stdin=cmd1.stdout, stdout=subprocess.PIPE)
        output = cmd2.stdout.read().decode()

        #self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600)

        print('Init :: soft gipper')

    def __del__(self):
        self.close_port()

    def grip(self):
        print('gripper grip')
        #c = c.encode(49)
        self.arduino.write(b'p')

    def suction(self):
        print('gripper suction')
        # c = c.encode(49)
        self.arduino.write(b's')
        #time.sleep(4)

    def open(self):
        print('gripper open')
        self.arduino.write(b'i')
        #time.sleep(4)

    def gripperSendMessage(self, c):
        c = c.encode('utf-8')
        self.arduino.write(49)

    def close_port(self):
        # Close port
        print('close')

if __name__ == "__main__":
    import time
    softGripper = Soft2PModeGripper()
    while True:
        softGripper.grip()
        time.sleep(.5)
        softGripper.open()
        time.sleep(1)
        softGripper.suction()
        time.sleep(1)
        softGripper.open()
        time.sleep(1)