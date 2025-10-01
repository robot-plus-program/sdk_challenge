from ketisdk.base.base import BasObj

def get_robot_default_cfg():
    from ketisdk.utils.proc_utils import CFG
    args = CFG()
    args.robot=CFG()
    args.robot.init_pose = [-0.53539, 0.00267, 0.56933, -2.2108, -2.2318, 0]
    args.robot.params = {'vel': 8, 'acc': 1.6}
    return args


class RobotProcess(BasObj):
    def __init__(self, args=None, cfg_path=None,
                 name='unnamed', default_args=None, **kwargs):
        BasObj.__init__(self, args=args, cfg_path=cfg_path, name=name, default_args=self.get_default_cfg())
        self.robot_start()
        self.gripper_start()
    def get_default_cfg(self):
        return get_robot_default_cfg()

    def gripper_start(self):
        print('gripper started')
    def robot_start(self, **kwargs):
        print('robot started')

    def robot_stop(self, **kwargs):
        print('robot stopped')

    def robot_translate_delta(self, delta, **kwargs):
        print('robot translated')

    def robot_movel(self, pose, **kwargs):
        print('robot move l')

    def robot_rotate(self, angle, **kwargs):
        print('robot rotated')
    def robot_move_init(self,**kwargs):
        print('robot moved init')

    def robot_approach_to_picking_location(self, **kwargs):
        print('Robot moved to picking location')

    def gripper_open(self, **kwargs):
        print('Gripper opened')
    def gripper_closed(self, **kwargs):
        print('Gripper closed')

    def gripper_release(self, **kwargs):
        print('Gripper Released')

    def robot_move_pick_to_placing_location(self, **kwargs):
        print('Robot moved to placing location')

    def test_robot_move(self):
        self.gripper_open()
        self.gripper_closed()
        self.gripper_release()
        d = 0.05
        self.robot_move_init()
        self.robot_translate_delta(delta=(d,0,0))
        self.robot_translate_delta(delta=(-2*d,0,0))
        self.robot_move_init()
        self.robot_translate_delta(delta=(0,d,0))
        self.robot_translate_delta(delta=(0,-2*d,0))
        self.robot_move_init()
        self.robot_translate_delta(delta=(0,0,d))
        self.robot_translate_delta(delta=(0,0,-2*d))
        self.robot_move_init()
        self.robot_rotate(30)
        self.robot_move_init()

    def run(self, **kwargs):
        self.gripper_open(**kwargs)
        self.robot_approach_to_picking_location(**kwargs)
        self.gripper_closed(**kwargs)
        self.robot_move_pick_to_placing_location(**kwargs)
        self.gripper_release(**kwargs)

    def gui_run(self, **kwargs):
        method_ind = kwargs['method_ind']
        if method_ind==0:
            self.robot_move_init(**kwargs)
        if method_ind==1:
            self.run(**kwargs)
        if method_ind==2:
            self.test_robot_move()

        return



