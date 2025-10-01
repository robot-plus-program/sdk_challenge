from ketisdk.sensor.zivid_sensor import ZividSensor
import cv2


def run():
    #========================== configs
    workspace=None
    depth_min=500
    dept_max=700
    disp_mode = 'rgb'
    #==========================

    rs = ZividSensor()
    rs.start()
    while True:
        try:
            rgbd = rs.get_rgbd(workspace=workspace,depth_min=depth_min, depth_max=dept_max)
            cv2.imshow('viewer', rgbd.disp(mode=disp_mode)[:,:,::-1])
            if cv2.waitKey(10)==27:
                break
        except:
            rs.stop()

def run_gui():
    from ketisdk.sensor.zivid_sensor import demo_run_zivid_gui
    demo_run_zivid_gui()


if __name__=='__main__':
    run_gui()