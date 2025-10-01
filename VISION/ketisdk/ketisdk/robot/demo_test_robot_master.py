import time

from robot.hanwha.hcr3 import Hcr3

if __name__ == '__main__':
    hcr3_robot = Hcr3('192.168.1.100', 5555, 'p0', 'ws0')

    while True:
        print('mv init')
        hcr3_robot.move_init2()
        print('mv init2')
        hcr3_robot.move_init()


