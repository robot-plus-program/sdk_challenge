#!/usr/bin/env python
# -*- coding: utf-8 -*-
# from pymodbus.client import ModbusTcpClient
from pymodbus.client.sync import ModbusTcpClient
import time
import threading


# Output data word 0 - 0x0801 (ControlWord)
# Output data word 1 - 0x0802 (DeviceMode, Workpiece No)
# Output data word 2 - 0x0803 (Reserve, PositionTolerance)
# Output data word 3 - 0x0804 (GripForce, DriveVelocity)
# Output data word 4 - 0x0805 (BasePosition)
# Output data word 5 - 0x0806 (ShiftPosition)
# Output data word 6 - 0x0807 (TeachPosition)
# Output data word 7 - 0x0808 (WorkPosition)

# Input data word 0 - 0x0002 (StatusWord)
# Input data word 1 - 0x0003 (Diagnosis)
# Input data word 2 - 0x0004 (ActualPosition)


class KetiZimmer:
    def __init__(self):
        self.NUM_RECV_REG = 3
        self.NUM_SEND_REG = 8
        self.ADDR_RECV = 0x0002
        self.ADDR_SEND = 0x0801

        self.MoveWorkpositionFlag = 0x4000
        self.MoveBasepositionFlag = 0x2000
        self.DataTransferOK = 0x1000
        self.AtWorkposition = 0x0400
        self.AtBaseposition = 0x0100
        self.PLCActive = 0x0040
        self.MovementComplete = 0x0008
        self.InMotion = 0x0004
        self.MotorOn = 0x0002
        self.HomingPositionOK = 0x0001

        self.ip = ''
        self.port = 0
        self.connected = False

        self.comm_thread_run = False
        self.comm_thread = threading.Thread(target=self.comm_func)

        self.mb = None
        self.reg_read = 0
        self.reg_write = [0, 0, 0, 0, 0, 0, 0, 0]

        self.gripper_force = 50
        self.gripper_velocity = 50
        self.max_distance = 4000
        self.grip_distance = 0

        self.send_flag = False
        self.comm_step = 0
        self.init_flag = False
        self.grip_flag = False
        self.release_flag = True

        self.mode = [85, 95]
        self.mode_indx = 0

        self.debug = False

    def connect(self, ip='192.168.0.253', port=502):
        self.mb = ModbusTcpClient(host=ip, port=port)
        self.connected = self.mb.connect()
        self.ip = ip
        self.port = port
        if self.connected is True:
            print('connected gripper')
            self.comm_thread.daemon = True
            self.comm_thread.start()
        else:
            print('not connected gripper')

    def disconnect(self):
        self.comm_thread_run = False
        self.mb.close()

    def comm_func(self):
        self.comm_thread_run = True

        while self.comm_thread_run is True and self.connected is True:
            self.reg_read = self.mb.read_input_registers(self.ADDR_RECV, self.NUM_RECV_REG, uint=16)
            self.grip_distance = self.reg_read.registers[2]
            # print('{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}'
            #       .format(self.reg_read.registers[0]&0x8000, self.reg_read.registers[0]&0x4000,
            #               self.reg_read.registers[0]&0x2000, self.reg_read.registers[0]&0x1000,
            #               self.reg_read.registers[0]&0x0800, self.reg_read.registers[0]&0x0400,
            #               self.reg_read.registers[0]&0x0200, self.reg_read.registers[0]&0x0100,
            #               self.reg_read.registers[0]&0x0080, self.reg_read.registers[0]&0x0040,
            #               self.reg_read.registers[0]&0x0020, self.reg_read.registers[0]&0x0010,
            #               self.reg_read.registers[0]&0x0008, self.reg_read.registers[0]&0x0004,
            #               self.reg_read.registers[0]&0x0002, self.reg_read.registers[0]&0x0001))
            # print(self.grip_distance)

            if self.send_flag is True:
                if self.comm_step == 0:
                    if bool(self.reg_read.registers[0] & self.PLCActive) is True:
                        if self.debug is True:
                            print("PLC active bit check complete")
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.comm_step = self.comm_step + 1

                elif self.comm_step == 1:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is True \
                            and bool(self.reg_read.registers[0] & self.MotorOn) is True:
                        if self.debug is True:
                            print("Data transfer ok bit & motor on bit check complete")
                        self.reg_write[0] = 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.comm_step = self.comm_step + 1

                elif self.comm_step == 2:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is not True:
                        if self.debug is True:
                            print('Handshake is done')
                        self.reg_write[0] = 1
                        self.reg_write[1] = self.mode[self.mode_indx] * 256 + 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.comm_step = self.comm_step + 1

                elif self.comm_step == 3:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is True:
                        if self.debug is True:
                            print('Data transfer ok bit check complete')
                        self.reg_write[0] = 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.comm_step = self.comm_step + 1
                        self.init_flag = True

                elif self.comm_step == 4:
                    # if bool(self.reg_read.registers[0]&self.DataTransferOK) is not True:
                    #     if self.grip_flag is True:
                    #         if self.debug is True:
                    #             print('grip move to workposition')
                    #         self.reg_write[0] = 512
                    #     else:
                    #         if bool(self.reg_read.registers[0]&self.AtBaseposition) is not True:
                    #             if self.debug is True:
                    #                 print('grip move to baseposition')
                    #             self.reg_write[0] = 256

                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is not True:
                        if self.grip_flag is True:
                            if self.mode_indx == 0:
                                if bool(self.reg_read.registers[0] & self.AtWorkposition) is not True:
                                    print('grip move to workposition')
                                    self.reg_write[0] = 512
                                else:
                                    print('grip position is workposition')
                                    self.send_flag = False
                            elif self.mode_indx == 1:
                                if bool(self.reg_read.registers[0] & self.AtBaseposition) is not True:
                                    print('grip move to baseposition')
                                    self.reg_write[0] = 256
                                else:
                                    print('grip position is baseposition')
                                    self.send_flag = False
                        else:
                            if self.mode_indx == 0:
                                if bool(self.reg_read.registers[0] & self.AtBaseposition) is not True:
                                    print('grip move to baseposition')
                                    self.reg_write[0] = 256
                                else:
                                    print('grip position is baseposition')
                                    self.send_flag = False
                            elif self.mode_indx == 1:
                                if bool(self.reg_read.registers[0] & self.AtWorkposition) is not True:
                                    print('grip move to workposition')
                                    self.reg_write[0] = 512
                                else:
                                    print('grip position is workposition')
                                    self.send_flag = False

                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.comm_step = self.comm_step + 1

                elif self.comm_step == 5:
                    if bool(self.reg_read.registers[0] & self.InMotion) is True \
                            and bool(self.reg_read.registers[0] & self.MovementComplete) is not True:
                        self.comm_step = self.comm_step + 1

                elif self.comm_step == 6:
                    if bool(self.reg_read.registers[0] & self.InMotion) is not True \
                            and bool(self.reg_read.registers[0] & self.MovementComplete) is True:
                        if self.debug is True:
                            print('move complete')
                        self.reg_write[0] = 4
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.comm_step = self.comm_step + 1

                elif self.comm_step == 7:
                    if bool(self.reg_read.registers[0] & self.MoveWorkpositionFlag) is not True \
                            and bool(self.reg_read.registers[0] & self.MoveBasepositionFlag) is not True:
                        self.comm_step = -1
                        self.send_flag = False

            time.sleep(0.01)
            # print(self.comm_step)

        self.reg_read.registers[0] = 0
        self.reg_read.registers[1] = 0
        self.reg_read.registers[2] = 0
        print('gripper modbus disconnected')

    def gripper_init(self):
        self.reg_write[0] = 1
        self.reg_write[1] = 3 * 256 + 0
        self.reg_write[2] = 50
        self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
        self.reg_write[4] = 100
        self.reg_write[5] = self.max_distance - 500
        self.reg_write[7] = self.max_distance

        self.init_flag = False
        self.comm_step = 0
        self.send_flag = True
        while self.init_flag is False:
            time.sleep(0.001)
        # print('Gripper Init')

    def gripper_grip(self, grip_distance=-1, sync=True):
        if grip_distance == -1:
            position = self.max_distance
        else:
            position = (74 - grip_distance) / 2 * 100 + 100

        if self.init_flag is True:
            self.reg_write[0] = 1
            self.reg_write[1] = 3 * 256 + 0
            self.reg_write[2] = 50
            self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
            self.reg_write[4] = 100
            self.reg_write[5] = position - 100
            self.reg_write[7] = position
            self.grip_flag = True

            self.comm_step = 0
            self.send_flag = True

            if sync is True:
                while self.send_flag is True:
                    time.sleep(0.001)

    def gripper_release(self, sync=True):
        if self.init_flag is True:
            self.reg_write[0] = 1
            self.reg_write[1] = 3 * 256 + 0
            self.reg_write[2] = 50
            self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
            self.reg_write[4] = 100
            self.reg_write[5] = self.max_distance - 100
            self.reg_write[7] = self.max_distance

            self.comm_step = 0
            self.send_flag = True
            self.grip_flag = False

            if sync is True:
                while self.send_flag is True:
                    time.sleep(0.001)

    def grip_custom(self, position=4000, velocity=50, force=50, sync=True):
        if self.init_flag is True:
            self.reg_write[0] = 1
            self.reg_write[1] = 3 * 256 + 0
            self.reg_write[2] = 50
            self.reg_write[3] = force * 256 + velocity

            if position > self.reg_read.registers[2]:
                self.reg_write[4] = 100
                self.reg_write[5] = position - 100
                self.reg_write[7] = position
                self.grip_flag = True
            else:
                self.reg_write[4] = position
                self.reg_write[5] = 2000
                self.reg_write[7] = self.max_distance
                self.grip_flag = False

            self.comm_step = 0
            self.send_flag = True

            if sync is True:
                while self.send_flag is True:
                    time.sleep(0.001)

    def grip_opt(self, velocity=50, force=50):
        self.gripper_velocity = velocity
        self.gripper_force = force

    def grip_get_pos(self):
        return self.grip_distance

    def grip_get_success(self):
        if self.grip_distance > self.reg_write[7] - self.reg_write[2]:
            return False
        else:
            return True

    def isConnected(self):
        return self.connected

    def gripper_jog_enable(self):
        self.reg_write[0] = 1
        self.reg_write[1] = 11 * 256 + 0
        self.reg_write[2] = 50
        self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
        self.reg_write[4] = 100
        self.reg_write[5] = 3000
        self.reg_write[7] = 4000

        self.mb.write_registers(self.ADDR_SEND, self.reg_write)

    def gripper_jog_plus(self):
        self.reg_write[0] = 1024
        self.reg_write[1] = 11 * 256 + 0
        self.reg_write[2] = 50
        self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
        self.reg_write[4] = 100
        self.reg_write[5] = 3000
        self.reg_write[7] = 4000

        self.mb.write_registers(self.ADDR_SEND, self.reg_write)

    def gripper_jog_minus(self):
        self.reg_write[0] = 2048
        self.reg_write[1] = 11 * 256 + 0
        self.reg_write[2] = 50
        self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
        self.reg_write[4] = 100
        self.reg_write[5] = 3000
        self.reg_write[7] = 4000

        self.mb.write_registers(self.ADDR_SEND, self.reg_write)

    def gripper_homing(self):
        self.reg_write[0] = 1
        self.reg_write[1] = 10 * 256 + 0
        self.reg_write[2] = 50
        self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
        self.reg_write[4] = 100
        self.reg_write[5] = 3000
        self.reg_write[7] = 4000

        self.mb.write_registers(self.ADDR_SEND, self.reg_write)

    def gripper_stop(self):
        self.reg_write[0] = 0
        self.reg_write[1] = 10 * 256 + 0
        self.reg_write[2] = 50
        self.reg_write[3] = self.gripper_force * 256 + self.gripper_velocity
        self.reg_write[4] = 100
        self.reg_write[5] = 3000
        self.reg_write[7] = 4000

        self.mb.write_registers(self.ADDR_SEND, self.reg_write)

    def set_inner(self):
        self.mode_indx = 1

    def set_outer(self):
        self.mode_indx = 0

    def grip_jog_grip(self,distance):
        self.gripper_jog_enable()
        time.sleep(0.1)
        self.gripper_jog_plus()
        while self.grip_distance <= distance:
            time.sleep(0.001)
        self.gripper_stop()
        self.gripper_jog_enable()
    def grip_jog_release(self,distance):
        self.gripper_jog_enable()
        time.sleep(0.1)
        self.gripper_jog_minus()
        while self.grip_distance >= distance:
            time.sleep(0.001)
        self.gripper_stop()
        self.gripper_jog_enable()
    def grip_in(self,distance=50,vel=0.1):
        target_distance=self.grip_distance + distance
        self.gripper_jog_enable()
        time.sleep(0.1)
        self.gripper_jog_plus()
        while self.grip_distance <= target_distance:
            time.sleep(vel)
        self.gripper_stop()
        self.gripper_jog_enable()
    def grip_out(self,distance=50):
        target_distance=self.grip_distance - distance
        self.gripper_jog_enable()
        time.sleep(0.1)
        self.gripper_jog_minus()
        while self.grip_distance >= target_distance:
            time.sleep(0.1)
        self.gripper_stop()
        self.gripper_jog_enable()

if __name__ == '__main__':
    gripper = KetiZimmer()
    gripper.connect('192.168.137.254', 502)

    gripper.gripper_init()
    gripper.gripper_grip()
    gripper.grip_in(100)
    gripper.grip_custom(1000)

    time.sleep(1)
    # print(gripper.grip_distance)

    gripper.gripper_release()
    gripper.gripper_grip()
    gripper.grip_out()
    time.sleep(1)
    print(gripper.grip_distance)
    gripper.grip_in()
    gripper.grip_jog_grip(gripper.grip_distance+100)