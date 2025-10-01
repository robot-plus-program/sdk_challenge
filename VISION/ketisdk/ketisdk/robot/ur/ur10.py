from ketisdk.robot.robot import RobotProcess
import urx
import numpy as np
import copy
import time


def get_ur10_default_cfg():
    from ketisdk.utils.proc_utils import CFG
    args = CFG()
    args.robot = CFG()
    args.robot.address = "192.168.0.7"
    args.robot.init_pose = [-0.53539, 0.00267, 0.66933, -2.2108, -2.2318, 0]
    args.robot.params = {'vel': 8, 'acc': 1.6, 'tool_size_mm': 210, 'min_z_axis_mm': 259,
                         'z_down': 0.1, 'z_up': 0.25}

    args.gripper = CFG()
    args.gripper.params = {'type': 'Soft2PModeGripper'}
    return args


def get_ur10_key_names():
    return ['robot.address', 'robot.init_pose', 'robot.params', 'gripper.params']


class UR10Process(RobotProcess):

    def get_default_cfg(self):
        return get_ur10_default_cfg()

    def robot_start(self, **kwargs):
        self.robot = urx.Robot(self.args.robot.address)

    def robot_move_init(self, **kwargs):
        self.robot.movel(self.args.robot.init_pose, acc=self.args.robot.params['acc'],
                         vel=self.args.robot.params['vel'])

    def robot_translate_delta(self, delta, **kwargs):
        self.robot.translate(delta, acc=self.args.robot.params['acc'], vel=self.args.robot.params['vel'])

    def robot_movel(self, pose, **kwargs):
        self.robot.movel(pose, acc=self.args.robot.params['acc'], vel=self.args.robot.params['vel'])

    def robot_rotate(self, angle, **kwargs):
        ur_transf = self.robot.get_pose()
        ur_transf.orient.rotate_zt((angle / 180.0) * np.pi)

        tmp_pose_vector = copy.deepcopy(ur_transf.pose_vector)
        self.robot.movel(tmp_pose_vector, acc=self.args.robot.params['acc'], vel=self.args.robot.params['vel'])

    def robot_approach_to_picking_location(self, **kwargs):
        detected = kwargs['detected']
        grip_xc, grip_yc, grip_zc, grip_w, grip_h, grip_angle, grip_x0, grip_y0, grip_x1, grip_y1, grip_prob = \
            detected['best'].flatten()
        rgbd = detected['rgbd']
        sensor_info = detected['sensor_info']

        dx = grip_x1 - grip_x0
        dy = grip_y1 - grip_y0
        dx1 = abs(dx)
        dy1 = abs(dy)

        robot_z_angle = (np.arctan2(dy1, dx1) / np.pi) * 180
        if dx < 0:
            robot_z_angle = 180 - robot_z_angle

        target_x = grip_xc
        target_y = grip_yc

        patch_size = 3
        # if grip_zc == 0:
        roi = rgbd.depth[int(grip_yc) - patch_size: int(grip_yc) + patch_size,
              int(grip_xc) - patch_size: int(grip_xc) + patch_size]
        grip_zc = np.min(roi[roi > 100])

        pz = grip_zc
        px = float((float(target_x) - sensor_info.cx)) / sensor_info.fx * float(pz)
        py = float((float(target_y) - sensor_info.cy)) / sensor_info.fy * float(pz)

        print("pxyz", px, py, pz)

        print("ur go to the picking points")

        calib_cam_cnt_from_ur_x = -77.5  # [mm]
        calib_cam_cnt_from_ur_y = -35.739  # [mm]

        robot_based_cam_x = py
        robot_based_cam_y = px

        robot_relative_x = (robot_based_cam_x + calib_cam_cnt_from_ur_x) / 1000.0
        robot_relative_y = (robot_based_cam_y + calib_cam_cnt_from_ur_y) / 1000.0

        self.robot.translate((robot_relative_x, robot_relative_y, 0), acc=self.args.robot.params['acc'],
                             vel=self.args.robot.params['vel'])

        ur_transf = self.robot.get_pose()
        ur_transf.orient.rotate_zt((robot_z_angle / 180.0) * np.pi)

        tmp_pose_vector = copy.deepcopy(ur_transf.pose_vector)
        # tmp_pose_vector[0] += cam_2_ur_x
        # tmp_pose_vector[1] += cam_2_ur_y
        # tmp_pose_vector[2] += -z_down
        self.robot.movel(tmp_pose_vector, acc=self.args.robot.params['acc'] * 2, vel=self.args.robot.params['vel'] * 3)
        # rob.set_pose(ur_transf, vel=v, acc=a)
        # rob.translate((robot_relative_x, robot_relative_y, 0), acc=a, vel=v)

        relative_z_trans = (-float(pz) / 1000.0) + (self.args.robot.params['tool_size_mm'] / 1000.0) - 0.015  # [m]
        relative_z_trans -= self.args.robot.params['z_down']
        self.robot.translate((0, 0, relative_z_trans), acc=self.args.robot.params['acc'], vel=self.args.robot.params['vel'])

        pose = self.robot.getl()

        # relative_z_trans = 0.238 - pose[2]  # [m]
        print("pz", pz)

    def robot_move_pick_to_placing_location(self, **kwargs):
        pose = self.robot.getl()

        pose_pickup = copy.deepcopy(pose)
        pose_pickup[2] += self.args.robot.params['z_up']

        r = 0.015

        # rob.movels([pose_pickup, place_pose1, place_down], vel=v, acc=a, radius=r)
        self.robot.movels([pose_pickup, ], vel=self.args.robot.params['acc'] / 4, acc=self.args.robot.params['vel'] / 4,
                          radius=r)


if __name__ == '__main__':
    aa = UR10Process()

    aa = 1
