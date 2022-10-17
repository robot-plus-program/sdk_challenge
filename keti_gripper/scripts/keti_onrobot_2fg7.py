#!/usr/bin/env python3

import xmlrpc.client
import time
import threading

TWOFG_ID = 0xC0
CONN_ERR = -2
RET_FAIL = -1
RET_OK = 0


class Device:
    cb = None
    cbip = None

    def __init__(self, cb_ip):
        self.cbip = cb_ip

    def get_cb(self):
        try:
            self.cb = xmlrpc.client.ServerProxy("http://" + str(self.cbip) + ":41414/")
            return self.cb
        except TimeoutError:
            print("Connection to ComputeBox failed!")


class TWOFG:
    # This class is for handling the 2FG device
    cb = None

    def __init__(self, dev, t_index):
        self.cb = dev.get_cb()
        self.gripper_status = 0
        # self.gripper_status_thread = threading.Thread(target=self.gripper_status_func, args = (t_index,), daemon=True)
        # self.gripper_status_thread.start()

    def isconn(self, t_index):
        """
        Returns with True if 2FG device is connected, False otherwise
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: True if connected, False otherwise
        @rtype: bool
        """
        IsTwoFG = self.cb.cb_is_device_connected(t_index, TWOFG_ID)
        if IsTwoFG is False:
            print("No 2FG device connected on the given instance")
            return False
        else:
            return True

    def isBusy(self, t_index):
        """
        Gets if the gripper is busy or not
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: bool
        @return: True if busy, False otherwise
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        return self.cb.twofg_get_busy(t_index)

    def isGripped(self, t_index):
        """
        Gets if the gripper is gripping or not
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: bool
        @return: True if gripped, False otherwise
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        return self.cb.twofg_get_grip_detected(t_index)

    def getStatus(self, t_index):
        """
        Gets the status of the gripper
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @rtype: int
        @return: Status code of the device
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        status = self.cb.twofg_get_status(t_index)
        return status

    def get_ext_width(self, t_index):
        """
        Returns with current external width
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: External width in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        extWidth = self.cb.twofg_get_external_width(t_index)
        return extWidth

    def get_int_width(self, t_index):
        """
        Returns with current internal width
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Internal width in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        intWidth = self.cb.twofg_get_internal_width(t_index)
        return intWidth

    def get_min_ext_width(self, t_index):
        """
        Returns with current minimum external width
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Minimum external width in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        extMinWidth = self.cb.twofg_get_min_external_width(t_index)
        return extMinWidth

    def get_min_int_width(self, t_index):
        """
        Returns with current minimum internal width
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Minimum internal width in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        intMinWidth = self.cb.twofg_get_min_internal_width(t_index)
        return intMinWidth

    def get_max_ext_width(self, t_index):
        """
        Returns with current maximum external width
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Maximum external width in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        extMaxWidth = self.cb.twofg_get_max_external_width(t_index)
        return extMaxWidth

    def get_max_int_width(self, t_index):
        """
        Returns with current maximum internal width
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Maximum internal width in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        intMaxWidth = self.cb.twofg_get_max_internal_width(t_index)
        return intMaxWidth

    def get_force(self, t_index):
        """
        Returns with current force
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Force in N
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        currForce = self.cb.twofg_get_force(t_index)
        return currForce

    def get_finger_len(self, t_index):
        """
        Returns with current finger length
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Finger length in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        fingerLength = self.cb.twofg_finger_length(t_index)
        return fingerLength

    def get_finger_height(self, t_index):
        """
        Returns with current finger height
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Finger height in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        fingerHeight = self.cb.twofg_finger_length(t_index)
        return fingerHeight

    def get_finger_orient(self, t_index):
        """
        Returns with current finger orientation
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Finger orinetation (1 inwards, 2 outwards)
        @rtype: int
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        fingerOrientation = self.cb.twofg_finger_orientation_outward(t_index)
        return fingerOrientation

    def get_fingertip_offset(self, t_index):
        """
        Returns with current fingertip offset
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @return: Fingertip offset in mm
        @rtype: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        fingertipOffset = self.cb.twofg_fingertip_offset(t_index)
        return fingertipOffset

    def halt(self, t_index):
        """
        Stop the grippers movement
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        self.cb.twofg_stop(t_index)

    def grip_int(self, t_index, t_width, n_force, p_speed, f_wait):
        """
        Makes an internal grip with the gripper to the desired position
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @param t_width: The width to move the gripper to in mm
        @type t_width: float
        @param n_force: The force to move the gripper width in N
        @type n_force: float
        @param p_speed: The speed of the gripper in %
        @type p_speed: int
        @type f_wait: bool
        @param f_wait: wait for the grip to end or not?
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        # Sanity check
        max = self.get_max_int_width(t_index)
        min = self.get_min_int_width(t_index)
        if t_width > max or t_width < min:
            print("Invalid 2FG width parameter, " + str(max)+" - "+str(min) +" is valid only")
            return RET_FAIL
        if n_force > 140 or n_force < 20:
            print("Invalid 2FG force parameter, 20-140 is valid only")
            return RET_FAIL
        if p_speed > 100 or p_speed < 10:
            print("Invalid 2FG speed parameter, 10-100 is valid only")
            return RET_FAIL
        self.cb.twofg_grip_internal(t_index, float(t_width), int(n_force), int(p_speed))
        if f_wait:
            tim_cnt = 0
            fbusy = self.isBusy(t_index)
            while (fbusy):
                time.sleep(0.1)
                fbusy = self.isBusy(t_index)
                tim_cnt += 1
                if tim_cnt > 30:
                    print("2FG internal grip command timeout")
                    break
                else:
                    #Grip detection
                    grip_tim = 0
                    gripped = self.isGripped(t_index)
                    while (not gripped):
                        time.sleep(0.1)
                        gripped = self.isGripped(t_index)
                        grip_tim += 1
                        if grip_tim > 20:
                            print("2FG internal grip detection timeout")
                            break
                        else:
                            return RET_OK
                    return RET_FAIL
            return RET_FAIL
        else:
            return RET_OK

    def grip_ext(self, t_index, t_width, n_force, p_speed, f_wait):
        """
        Makes an external grip with the gripper to the desired position
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @param t_width: The width to move the gripper to in mm
        @type t_width: float
        @param n_force: The force to move the gripper width in N
        @type n_force: float
        @param p_speed: The speed of the gripper in %
        @type p_speed: int
        @type f_wait: bool
        @param f_wait: wait for the grip to end or not?
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        # Sanity check
        max = self.get_max_ext_width(t_index)
        min = self.get_min_ext_width(t_index)
        if t_width > max or t_width < min:
            print("Invalid 2FG width parameter, " + str(max)+" - "+str(min) + " is valid only")
            return RET_FAIL
        if n_force > 140 or n_force < 20:
            print("Invalid 2FG force parameter, 20-140 is valid only")
            return RET_FAIL
        if p_speed > 100 or p_speed < 10:
            print("Invalid 2FG speed parameter, 10-100 is valid only")
            return RET_FAIL
        self.cb.twofg_grip_external(t_index, float(t_width), int(n_force), int(p_speed))
        self.gripper_status = self.getStatus(t_index)
        if f_wait:
            tim_cnt = 0
            while True:
                time.sleep(0.1)
                self.gripper_status = self.getStatus(t_index)
                dist = abs(float(t_width) - self.get_ext_width(t_index))
                if dist < 0.1:
                    return RET_OK
                tim_cnt += 1
                if tim_cnt > 30:
                    print("2FG external grip command timeout")
                    return RET_FAIL
                else:
                    # Grip detection
                    grip_tim = 0
                    while True:
                        time.sleep(0.1)
                        gripped = self.isGripped(t_index)
                        if gripped:
                            return RET_OK
                        grip_tim += 1
                        if grip_tim > 30:
                            print("2FG external grip detection timeout")
                            break
        else:
            return RET_OK

    def move(self, t_index, t_width, f_wait):
        """
        Moves the gripper to the desired position
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @param t_width: The width to move the gripper to in mm
        @type t_width: float
        @type f_wait: bool
        @param f_wait: wait for the grip to end or not?
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        max = self.get_max_ext_width(t_index)
        min = self.get_min_ext_width(t_index)
        if t_width > max or t_width < min:
            print("Invalid 2FG diameter parameter, " + str(max)+" - "+str(min) + " is valid only")
            return RET_FAIL
        self.cb.twofg_grip_external(t_index, float(t_width), 100, 80)
        self.gripper_status = self.getStatus(t_index)
        if f_wait:
            tim_cnt = 0
            while True:
                time.sleep(0.1)
                self.gripper_status = self.getStatus(t_index)
                dist = abs(float(t_width) - self.get_ext_width(t_index))
                if dist < 0.1:
                    return RET_OK
                tim_cnt += 1
                if tim_cnt > 30:
                    print("2FG external grip command timeout")
                    return RET_FAIL
        else:
            return RET_OK

    def set_finger_len(self, t_index, flen):
        """
        Sets the finger lenght of the gripper
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param flen: Finger lenght in mm
        @type   flen: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        if flen > 100 or flen < 0:
            print("Invalid 2FG finger length parameter, 0-100 is valid only")
            return RET_FAIL
        self.cb.twofg_set_finger_length(t_index, float(flen))

    def set_finger_height(self, t_index, fh):
        """
        Sets the finger height of the gripper
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param fh: Finger height in mm
        @type   fh: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        if fh > 100 or fh < 0:
            print("Invalid 2FG finger height parameter, 0-100 is valid only")
            return RET_FAIL
        self.cb.twofg_set_finger_height(t_index, float(fh))

    def set_ft_offset(self, t_index, foffs):
        """
        Sets the fingertip offset of the gripper
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param foffs: Fingertip offset in mm
        @type foffs: float
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        if foffs > 100 or foffs < 0:
            print("Invalid 2FG fingertip offset parameter, 0-100 is valid only")
            return RET_FAIL
        self.cb.twofg_set_fingertip_offset(t_index, float(foffs))

    def set_finger_orient(self, t_index, f_orient):
        """
        Sets the finger orinetation of the gripper
        @param t_index: The position of the device (0 for single, 1 for dual primary, 2 for dual secondary)
        @type t_index: int
        @param f_orient: Finger orinetation (1 inward, 2 outward)
        @type f_orient: int
        """
        if self.isconn(t_index) is False:
            return CONN_ERR
        if type(f_orient) != bool:
            print("Invalid 2FG inward/outward, boolean is valid only")
            return RET_FAIL
        self.cb.twofg_set_finger_orientation(t_index, float(f_orient))

    def gripper_status_func(self, t_index):
        while True:
            time.sleep(0.1)
            # self.gripper_status = self.getStatus(t_index)
            print("status : " + str(self.gripper_status))


if __name__ == '__main__':
    twofg = TWOFG(dev=Device(cb_ip='192.168.137.201'), t_index=0)

    twofg.grip_ext(t_index=0, t_width=1, n_force=20, p_speed=10, f_wait=True)
    print("grip detection : " + str(twofg.isGripped(t_index=0)))

    ext_width = twofg.get_ext_width(t_index=0)
    force = twofg.get_force(t_index=0)

    print("ext width : " + str(ext_width))
    print("grip force : " + str(force))

    twofg.move(t_index=0, t_width=39, f_wait=True)

    ext_width = twofg.get_ext_width(t_index=0)
    print("ext width : " + str(ext_width))

    twofg.gripper_status = twofg.getStatus(t_index=0)
    time.sleep(0.5)

