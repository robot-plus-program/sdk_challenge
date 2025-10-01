from ketisdk.robot.robot_gui import RobotGui
from ketisdk.base.base import BasObj
from ketisdk.base.base_tcp import ServerThread
import os, time
import json
import numpy as np
import cv2
from ketisdk.gui.default_config import default_args
from ketisdk.utils.proc_utils import CFG
import kpick

KPICK_DIR = os.path.split(kpick.__file__)[0]
HOME_DIR = os.path.expanduser("~")

BASE_CFG = default_args()
BASE_CFG.path.shared_dir = os.path.join(HOME_DIR, '000_yumi_shared')

BASE_CFG.calib = CFG()
BASE_CFG.calib.intr_path = os.path.join(KPICK_DIR, 'apps/fpcb/parameter/zivid_intrinsic.json')
BASE_CFG.calib.left_affine_path = os.path.join(KPICK_DIR, 'apps/fpcb/parameter/yumi_zivid_affine/left_10_220117.npy')
BASE_CFG.calib.right_affine_path = os.path.join(KPICK_DIR, 'apps/fpcb/parameter/yumi_zivid_affine/right_10.json')

BASE_CFG.move = CFG()
BASE_CFG.move.move_delta_r = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
BASE_CFG.move.move_delta_l = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
BASE_CFG.move.move_r = [0.3853, -0.4253, 0.1507, 176.6825, -1.6426, 83.7239]
BASE_CFG.move.move_l = [0.3853, 0.4253, 0.1507, 176.4856, 1.1283, 87.1275]
BASE_CFG.move.left_calib_compensate = [0.0, 0.0, 0.0]
BASE_CFG.move.right_calib_compensate = [0.0, 0.0, -0.02]
BASE_CFG.move.default_rot_r = [176.6825, -1.6426, 83.7239]
BASE_CFG.move.default_rot_l = [176.4856, 1.1283, 87.1275]

BASE_CFG.ws = CFG()
BASE_CFG.ws.proj = 'unknown'
BASE_CFG.ws.workspace = 'undefined'
BASE_CFG.ws.floor = None


class YumiClient():
    def load_params(self, shared_dir='data/yumi_shared',
                    default_rot_r=[0.01187, 0.74463, 0.66669, 0.03018],
                    default_rot_l=[0.00869, 0.99626, -0.08031, 0.03051],
                    intr_path=None, left_affine_path=None, right_affine_path=None):
        self.shared_dir = shared_dir
        os.makedirs(self.shared_dir, exist_ok=True)
        self.req_file = os.path.join(self.shared_dir, 'req.json')
        self.res_file = os.path.join(self.shared_dir, 'res.json')

        self.default_rot_r = default_rot_r
        self.default_rot_l = default_rot_l

        # calibration
        self.offset = [0, 0, 0]
        if intr_path is not None:
            if os.path.exists(intr_path):
                with open(intr_path, "r") as st_json:
                    intrinsic_param = json.load(st_json)
            else:
                print(f'{intr_path} does not exist...')
            self.camera_matrix = np.array(intrinsic_param["cameraMatrix"])
        if left_affine_path is not None:
            if os.path.exists(left_affine_path):
                try:
                    affine = open(left_affine_path, "r").load(st_json)
                    self.affine_left = np.array(affine["affine"])
                except:
                    self.affine_left = np.load(left_affine_path)
                print(f'{left_affine_path} loadded ...')
            else:
                print(f'{left_affine_path} does not exist...')
        if right_affine_path is not None:
            if os.path.exists(right_affine_path):
                try:
                    affine = open(right_affine_path, "r").load(st_json)
                    self.affine_right = np.array(affine["affine"])
                except:
                    self.affine_right = np.load(right_affine_path)
                print(f'{right_affine_path} loadded ...')
            else:
                print(f'{right_affine_path} does not exist...')

    def printList(self, aList):
        s = ''
        for name in aList:
            s += '{}: {}\n'.format(name, aList[name])
        print(s)

    def im2robotLoc(self, loc):
        arm_val = np.matmul(self.R, np.array(loc).reshape((-1, 1))) + self.T
        return arm_val.flatten().tolist()

    def robot2imLoc(self, loc):
        im_val = np.matmul(self.invR, np.array(loc).reshape((-1, 1)) - self.T)
        return im_val.flatten().tolist()

    def goto_init(self):
        req = {'lb': 'GOTO-INIT'}
        self.send_req(req)
        self.get_res()

    def goto_safe_box(self):
        reqs = []
        # reqs.append(self.get_move_loc_req(r_loc=[0.18,-0.2,0.032]))
        # reqs.append(self.get_move_loc_req(r_loc=[0.48,-0.2,0.032]))
        # reqs.append(self.get_move_loc_req(r_loc=[0.18,-0.2,0.1]))
        # reqs.append(self.get_move_loc_req(r_loc=[0.48,-0.2,0.1]))

        reqs.append(self.get_move_loc_req(r_loc=[0.18, 0., 0.032]))
        reqs.append(self.get_move_loc_req(r_loc=[0.48, 0., 0.032]))
        reqs.append(self.get_move_loc_req(r_loc=[0.18, 0., 0.1]))
        reqs.append(self.get_move_loc_req(r_loc=[0.48, 0., 0.1]))

        # X,Y, Z = np.meshgrid(self.args.net.safe_x, self.args.net.safe_y, self.args.net.safe_z)
        # for right_arm in [True, False]:
        #     for x,y,z in zip(X.flatten(), Y.flatten(), Z.flatten()):
        #         req = self.get_move_loc_req(r_loc=[x,y,z]) if right_arm else self.get_move_loc_req(l_loc=[x,y,z])
        #         reqs.append(req)
        seq_req = {'lb': 'GOTO-SAFE-BOX', 'cmd': 'SEQ', 'seq': reqs}
        self.send_req(seq_req)

    def goto(self, r_pose=(0., 0., 0., 0., 0., 0.), l_pose=(0., 0., 0., 0., 0., 0.),
             pick=False, get_rep=True):
        lb = 'GOTO' if not pick else 'GOTO-PICK'
        req = {'lb': lb, 'r_pose': r_pose, 'l_pose': l_pose}
        self.send_req(req)
        if get_rep:
            return self.get_res()
            

    def goto_delta(self, r_dpose=(0., 0., 0., 0., 0., 0.), l_dpose=(0., 0., 0., 0., 0., 0.)):
        req = {'lb': 'GOTO-DELTA', 'r_dpose': r_dpose, 'l_dpose': l_dpose}
        self.send_req(req)
        self.get_res()

    def send_req(self, req):
        with open(self.req_file, 'w') as f: json.dump(req, f)
        print('REQUEST sent ...')
        self.printList(req)

    def get_res(self):
        print('Scaning RES file ... ')
        while True:
            time.sleep(1)
            if not os.path.exists(self.res_file): continue
            try:
                with open(self.res_file, 'r') as f:
                    res = json.load(f)
                    os.remove(self.res_file)
                print(f'RESPOND received ...')
                self.printList(res)
                return res
            except:
                pass
            return

    def get_state(self):
        req = {'lb': 'GET-STATE'}
        self.send_req(req)
        self.get_res()

    def Camera2Robot3d(self, CameraPosx, CameraPosy, CameraDepthZ, affine3dMatrix):
        Robot_Posx = affine3dMatrix[0][0] * CameraPosx + affine3dMatrix[0][1] * CameraPosy + \
                     affine3dMatrix[0][2] * CameraDepthZ + affine3dMatrix[0][3]

        Robot_Posy = affine3dMatrix[1][0] * CameraPosx + affine3dMatrix[1][1] * CameraPosy + \
                     affine3dMatrix[1][2] * CameraDepthZ + affine3dMatrix[1][3]

        Robot_Posz = affine3dMatrix[2][0] * CameraPosx + affine3dMatrix[2][1] * CameraPosy + \
                     affine3dMatrix[2][2] * CameraDepthZ + affine3dMatrix[2][3]

        return Robot_Posx, Robot_Posy, Robot_Posz

    def pxDepth2Robot3D(self, camera_x_2d, camera_y_2d, camera_z, rightArm=True, floor=None):
        points_c = [(camera_x_2d - self.camera_matrix[0][2]) / self.camera_matrix[0][0] * camera_z,
                    (camera_y_2d - self.camera_matrix[1][2]) / self.camera_matrix[1][1] * camera_z,
                    camera_z]
        points_c = np.array(points_c)
        affine = self.affine_right if rightArm else self.affine_left
        calRobotX, calRobotY, calRobotZ = self.Camera2Robot3d(points_c[0], points_c[1], points_c[2], affine)
        calRobotX = float(calRobotX - self.offset[0]) / 1000
        calRobotY = float(calRobotY - self.offset[1]) / 1000
        calRobotZ = float(calRobotZ - self.offset[2]) / 1000

        calRobotZ -= 0.01  # move down liite bit
        if floor is not None:  # interpolate
            dd = np.copy(floor[:, :2])
            dd[:, 0] -= calRobotX
            dd[:, 1] -= calRobotY
            dd = np.linalg.norm(dd, axis=1, keepdims=True)
            dd = np.sum(dd) - dd
            ww = dd / np.sum(dd)
            z_min = np.sum(np.multiply(floor[:, [2]], ww)) + 0.002
            print(f'z_min: {z_min}, z: {calRobotZ}')
            calRobotZ = max(z_min, calRobotZ)

        return [round(calRobotX, 5), round(calRobotY, 5), round(calRobotZ, 5)]

    def goto_loc(self, x, y, depth, calib_compensate=(0, 0, 0), dz_at_pick=(0, 0),
                 pick=False, rightArm=True, floor=None, z_angle=None, get_rep=True):

        # z = rgbd.depth[(y, x)]
        r = 5
        depth_roi = depth[y - r:y + r, x - r:x + r]
        z = np.amin(depth_roi[np.where(depth_roi > 100)])

        rx, ry, rz = self.pxDepth2Robot3D(x, y, z, floor=floor)
        dx, dy, dz = calib_compensate
        rxc, ryc, rzc = rx + dx, ry + dy, rz + dz
        print(f'>>> Image: [{x},{y},{z}] \t --> Robot: [{rx},{ry},{rz}]  --> Compensated: [{rxc}, {ryc},{rzc}]')
        pose = [rxc, ryc, rzc] + self.default_rot_r
        r_pose = pose if rightArm else None
        l_pose = pose if not rightArm else None
        if z_angle is not None:
            # if z_angle<0: z_angle += 360
            if rightArm:
                r_pose[-1] = z_angle
            else:
                l_pose[-1] = z_angle
        return  self.goto(l_pose=l_pose, r_pose=r_pose, pick=pick, get_rep=get_rep)

    def goto_click(self, rgbd, click_locs, calib_compensate=(0, 0, 0), dz_at_pick=(0, 0),
                   pick=False, rightArm=True, floor=None):
        print('click to select loc to forward ...')
        last_num_click = len(click_locs)
        while True:
            time.sleep(1)
            num_click = len(click_locs)
            if num_click == last_num_click: continue
            x, y = click_locs[-1]
            self.goto_loc(x, y, rgbd.depth, calib_compensate=calib_compensate,
                          dz_at_pick=dz_at_pick, pick=pick, rightArm=rightArm, floor=floor)
            break

    def goto_detected(self, rgbd, detected, calib_compensate=(0, 0, 0), dz_at_pick=(0, 0),
                      pick=False, rightArm=True, floor=None):
        if detected is None:
            print('No detected found ...')
            return

        grasp = detected['grasp']
        if grasp is None:
            print('No grasp found ...')
            return

        x, y, angle = grasp[:3]
        print(f'==============Detected Grasp: ({x}, {y}, {angle})')

        angle = 90 -angle
        if angle<0: angle+= 180

        return self.goto_loc(x, y, depth=rgbd.depth, rightArm=False, pick=True, floor=floor, z_angle=angle,
                             calib_compensate=calib_compensate)

    def open_close_gripper(self, rightArm=True):
        lb = 'GRIPER'
        req = {'lb': lb, 'rightArm': rightArm}
        self.send_req(req)
        self.get_res()


class YumiClientGui(YumiClient, RobotGui):

    def __init__(self, args=None, cfg_path=None, name='unnamed'):
        super().__init__(args=args, cfg_path=cfg_path, name=name, default_args=BASE_CFG)
        self.floor = np.array(self.args.ws.floor)

    def load_params(self, args):
        RobotGui.load_params(self, args=args)
        YumiClient.load_params(self, shared_dir=self.args.path.shared_dir,
                               default_rot_r=self.args.move.default_rot_r,
                               default_rot_l=self.args.move.default_rot_l,
                               intr_path=self.args.calib.intr_path,
                               left_affine_path=self.args.calib.left_affine_path,
                               right_affine_path=self.args.calib.right_affine_path)

    def gui_run(self, rgbd=None, method_ind=0, input=None, click_locs=None, **kwargs):
        if method_ind == 8: self.get_state()
        if method_ind == 1: self.open_close_gripper(rightArm=False)
        if method_ind == 2: self.open_close_gripper(rightArm=True)
        if method_ind == 3: self.goto_init()
        if method_ind == 4: self.goto_delta(r_dpose=self.args.move.move_delta_r, l_dpose=self.args.move.move_delta_l)
        if method_ind == 5: self.goto(r_pose=self.args.move.move_r, l_pose=self.args.move.move_l)
        if method_ind == 6: self.goto_click(rgbd=rgbd, click_locs=click_locs,
                                            calib_compensate=self.args.move.left_calib_compensate,
                                            rightArm=False, floor=self.floor)
        if method_ind == 7: self.goto_click(rgbd=rgbd, click_locs=click_locs,
                                            calib_compensate=self.args.move.left_calib_compensate,
                                            pick=True, rightArm=False, floor=self.floor)
        if method_ind == 0: self.goto_detected(rgbd=rgbd, detected=input,
                                               calib_compensate=self.args.move.left_calib_compensate)


def run_yumi_client_gui(cfg_path=None, sensor_mudules=[]):
    from ketisdk.gui.gui import GUI, GuiModule
    robot = GuiModule(YumiClientGui, type='ABB_robot', name='YuMi', category='robot_arm', cfg_path=cfg_path,
                      num_method=12)
    GUI(title='YuMi Controller', modules=[robot, ] + sensor_mudules)


def demo_affine_transform():
    import cv2
    import numpy as np
    from ketisdk.vision.utils.rgbd_utils_v2 import RGBD
    im_vals = [(757, 98, 701), (778, 536, 710), (206, 566, 711), (181, 125, 706)]
    arm_vals = [(499, -145, 31), (167, -163, 12), (158, 274, 13), (471, 286, 26)]

    src = np.array(im_vals)
    dst = np.array(arm_vals)

    RT = cv2.estimateAffine3D(src, dst)[1]
    R, T = RT[:, :-1], RT[:, -1].reshape((-1, 1))

    vi = src[-1].reshape((-1, 1))

    vo = np.matmul(R, vi) + T

    # aa = cv2.getAffineTransform(np.array(im_vals), np.array(arm_vals))
    aa = 1

    pass


if __name__ == '__main__':
    YumiClientGui()
