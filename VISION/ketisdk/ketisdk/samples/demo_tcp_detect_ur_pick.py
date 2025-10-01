
def main():
    from ketisdk.sensor.realsense_sensor import get_realsense_modules
    from ketisdk.robot.ur.ur10 import UR10Process, get_ur10_key_names
    from ketisdk.gui.gui import GUI, GuiModule

    robot_module = GuiModule(UR10Process, type='UR_robot', name='UR10', category='robot', num_method=3,
                             key_args=get_ur10_key_names(), cfg_path='configs/ur10.cfg')

    GUI(title='Demo TCP Grasp Detection for Robot Picking', modules=[robot_module, ]+get_realsense_modules(),
        default_cfg_path='configs/defaults.cfg', add_tcp_client=True, processes=[1.0, 0.0, 1.1])

if __name__=='__main__':
    main()