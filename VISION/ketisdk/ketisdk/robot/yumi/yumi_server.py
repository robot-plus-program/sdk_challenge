
class YumiServer():
    def __init__(self, shared_dir='data/yumi_shared', z_low=0.05, z_lowest=0.0):
        self.z_low=z_low
        self.z_lowest=z_lowest
        self.shared_dir = shared_dir
        if not os.path.exists(self.shared_dir): os.makedirs(self.shared_dir)
        self.req_file = os.path.join(self.shared_dir, 'req.json')
        self.res_file = os.path.join(self.shared_dir, 'res.json')

        self.constants = yumipy.YuMiConstants()
        self.robot = yumipy.YuMiRobot()
        self.state = yumipy.YuMiState()

        self.robot.calibrate_grippers()
        # self.robot.open_grippers()
        # self.robot.right.close_gripper()
        # self.robot.left.close_gripper()

        # go to init pose
        self.prevent_bottom_collision()
        self.gotoState(self.constants.R_READY_STATE, rightArm=True)
        self.gotoState(self.constants.L_READY_STATE, rightArm=False)


    def printList(self, aList):
        s = ''
        for name in aList:
            s += '{}: {}\n'.format(name, aList[name])
        print(s)

    def gotoState(self, state, rightArm=True):
        arm = self.robot.right if rightArm else self.robot.left
        # pose = arm.get_pose()
        # pose.translation[2] += 0.01
        # arm.goto_pose(pose, linear=True, relative=True)
        arm.goto_state(state)
        time.sleep(0.1)

    def getState(self, rightArm=True):
        arm = self.robot.right if rightArm else self.robot.left
        # state = arm.get_state()
        pose = arm.get_pose()
        gripWidth = round(arm.get_gripper_width(), ndigits=4)
        trans = np.round(pose.translation.tolist(), decimals=4).tolist()
        rot = np.round(pose.euler.tolist(), decimals=4).tolist()

        # aa = pose.quaternion
        # bb = transformations.euler_from_quaternion(pose.quaternion)
        # bb = np.array(bb)*180/np.pi

        # ai, aj, ak = np.array(rot)/180*np.pi
        # rot_= transformations.quaternion_from_euler(ai, aj, ak, 'szyx')
        # pose_ = RigidTransform(translation=trans, rotation=rot_)

        ret = {'pose': trans + rot}
        return ret

    # def move_delta(self, dpose, rightArm=True, show_validation=True):
    #     arm = self.robot.right if rightArm else self.robot.left
    #     pose = arm.get_pose()
    #     trans_prev = pose.translation
    #     rot_prev = pose.euler
    #
    #     dpose = np.round(dpose, decimals=4).tolist()
    #     if show_validation:
    #         print('SHIFT >>>')
    #         print('\t dpose: {}'.format(dpose))
    #     dy, dx, dz, ax, ay, az = dpose
    #     arm.goto_pose_delta(translation=(dx, dy, -dz), rotation=(ax, -ay, -az))
    #     time.sleep(0.1)
    #
    #     pose = arm.get_pose()
    #     dtrans = np.round((pose.translation - trans_prev).tolist(), decimals=4).tolist()
    #     drot = np.round((pose.euler - rot_prev).tolist(), decimals=4).tolist()
    #     if show_validation:
    #         print('  VALIDATION:')
    #         print('\t dpose: {}'.format(dtrans + drot))
    #
    #     return dtrans+drot

    def goto_delta(self, dpose, rightArm=True, show_validation=True):

        if dpose is None: return
        if np.amax(np.abs(dpose)) < 0.001: return

        arm = self.robot.right if rightArm else self.robot.left
        pose = arm.get_pose()
        trans_prev = pose.translation
        rot_prev = pose.euler

        dpose = np.round(dpose, decimals=4).tolist()
        if show_validation:
            print('SHIFT >>>')
            print('\t dpose: {}'.format(dpose))
        # dy, dx, dz, ax, ay, az = dpose
        # arm.goto_pose_delta(translation=(dx, dy, -dz), rotation=(ax, -ay, -az))
        dx, dy, dz, ax, ay, az = dpose
        # arm.goto_pose_delta(translation=(0.0, 0.0, 0.0), rotation=(ax, -ay, -az))
        # dx, dy, dz, ax, ay, az = dpose
        # pose = arm.get_pose()
        # pose.translation[0] += dx
        # pose.translation[1] += dy
        # pose.translation[2] += dz
        # pose.euler[0] += ax
        # pose.euler[1] += ay
        # pose.euler[2] += az
        target_trans = (trans_prev[0]+dx, trans_prev[1]+dy, trans_prev[2]+dz)
        target_rot = theta2quaternion(rot_prev[0]+ax,rot_prev[1]+ay, rot_prev[2]+az)
        target_pose = RigidTransform(translation=target_trans,
                                     rotation=target_rot)
        arm.goto_pose(target_pose, linear=True)

        time.sleep(0.1)

        pose = arm.get_pose()
        dtrans = np.round((pose.translation - trans_prev).tolist(), decimals=4).tolist()
        drot = np.round((pose.euler - rot_prev).tolist(), decimals=4).tolist()
        if show_validation:
            print('  VALIDATION:')
            print('\t dpose: {}'.format(dtrans + drot))

    def goto(self, pose, rightArm=True):
        # ai, aj, ak = np.array(rot)/180*np.pi
        # pose = RigidTransform(translation=trans,
        #                       rotation=transformations.quaternion_from_euler(ai, aj, ak, 'szyx'))
        # arm = self.robot.right if rightArm else self.robot.left
        # arm.goto_pose(pose)
        # time.sleep(0.1)
        if pose is None: return
        if self.is_collide_pose(pose): return
        arm = self.robot.right if rightArm else self.robot.left

        target_rot = theta2quaternion(pose[3], pose[4], pose[5])
        target_pose = RigidTransform(translation=pose[:3], rotation=target_rot)

        # if self.is_unreachable_pose(target_pose): return
        arm.goto_pose(target_pose, linear=True,relative=True)

        # current_pose = arm.get_pose()
        # current_pose = current_pose.translation.tolist() + current_pose.euler.tolist()
        # dpose = (np.array(current_pose) - np.array(pose)).tolist()
        # self.move_delta(dpose, rightArm=rightArm)

        tpose = arm.get_pose()
        tpose = tpose.translation.tolist() + tpose.euler.tolist()
        diff = np.round(np.array(tpose) - np.array(pose), 3)
        lb = 'right' if rightArm else 'left'
        print('MOVE VALIDATION: ')
        print('\t {}: reached:\t {}'.format(lb, np.round(tpose, 3).tolist()))
        print('\t\t error: \t {}'.format(diff))

        # self.move_delta_loop_fix(dpose=dpose, rightArm=rightArm)

    def goto_pick(self, pose, dz_at_pick=(0,0), rightArm=True):
        if pose is None: return
        arm = self.robot.right if rightArm else self.robot.left
        up, down = dz_at_pick

        pose[2] += up
        # if self.is_unreachable_pose(pose): return
        self.goto(pose,rightArm=rightArm)
        arm.open_gripper()

        pose[2] += down - up
        pose[2] = max(self.z_lowest, pose[2])
        # if self.is_unreachable_pose(pose): return
        self.goto(pose, rightArm=rightArm)
        arm.close_gripper()

        self.move_z(0.2, rightArm=rightArm)

    def prevent_bottom_collision(self):
        if self.is_low_pose(self.getState(rightArm=True)['pose']):
            self.move_z(0.1, rightArm=True)

        if self.is_low_pose(self.getState(rightArm=False)['pose']):
            self.move_z(0.1, rightArm=False)

    def move_z(self, dz, rightArm=True):
        pose = self.getState(rightArm=rightArm)['pose']
        pose[2] += dz
        self.goto(pose, rightArm=rightArm)

    def is_collide_pose(self, pose):
        pose = self.to_pose(pose)
        is_collide = pose[2]<self.z_lowest
        if is_collide: print('>>> collided pose: z = {}'.format(pose[2]))
        return is_collide

    def is_low_pose(self, pose):
        pose = self.to_pose(pose)
        return (pose[2] < self.z_low)

    def is_unreachable_pose(self, pose, rightArm=True):
        arm = self.robot.right if rightArm else self.robot.left
        is_unreach = arm.is_pose_reachable(self.to_RigidTransform(pose))
        if is_unreach: print('>>> pose is unreachable ...')
        return is_unreach

    def to_pose(self, pose):
        if isinstance(pose, RigidTransform):
            pose = pose.translation.tolist() + pose.euler.tolist()
        return pose

    def to_RigidTransform(self, pose):
        if not isinstance(pose, RigidTransform):
            rot = theta2quaternion(pose[3], pose[4], pose[5])
            pose = RigidTransform(translation=pose[:3], rotation=rot)
        return pose

    def do_req(self, req):
        self.prevent_bottom_collision()
        res = req
        if req['lb'] == 'GOTO-INIT':
            self.gotoState(self.constants.R_READY_STATE, rightArm=True)
            self.gotoState(self.constants.L_READY_STATE, rightArm=False)
        elif req['lb'] == 'GET-STATE':
            res_r = self.getState(rightArm=True)
            res_l = self.getState(rightArm=False)
            res = {'left': res_l, 'right': res_r}
        elif req['lb'] == 'GOTO-DELTA':
            self.goto_delta(dpose=req['r_dpose'], rightArm=True)
            self.goto_delta(dpose=req['l_dpose'], rightArm=False)
        elif req['lb'] == 'GOTO':
            self.goto(pose=req['r_pose'], rightArm=True)
            self.goto(pose=req['l_pose'], rightArm=False)
        elif req['lb'] == 'GOTO-PICK':
            self.goto_pick(pose=req['r_pose'], dz_at_pick=req['dz_at_pick'], rightArm=True)
            self.goto_pick(pose=req['l_pose'], dz_at_pick=req['dz_at_pick'], rightArm=False)
        elif req['lb'] == 'OPEN':
            arm = self.robot.right if req['rightArm'] else self.robot.left
            arm.open_gripper()
        elif req['lb'] == 'CLOSE':
            arm = self.robot.right if req['rightArm'] else self.robot.left
            arm.close_gripper()

        res_r = self.getState(rightArm=True)
        res_l = self.getState(rightArm=False)
        res = {'left': res_l, 'right': res_r}

        self.send_res(res)

    def send_res(self, res):
        with open(self.res_file, 'w') as f: json.dump(res, f)
        # print('RESPOND sent ... \n {}'.format(res))
        print('RESPOND sent ... \n')

    def get_req(self):
        print('Scaning REQ file ... ')
        while True:
            time.sleep(1)
            if not os.path.exists(self.req_file): continue
            with open(self.req_file, 'r') as f:
                req = json.load(f)
            os.remove(self.req_file)
            print('REQUEST received ...')
            self.printList(req)
            return req

    def run(self):
        while True:
            print("+" * 50)
            req = self.get_req()
            self.do_req(req)

def run_yumi_server():
    robot = YumiServer()
    robot.run()

if __name__ == '__main__':
    run_yumi_server()
    # demo_affine_transform()
