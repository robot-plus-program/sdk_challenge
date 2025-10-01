from ketisdk.sensor.realsense_sensor import RSSensor
import cv2

#========================== configs
workspace=None
depth_min=500
dept_max=700
disp_mode = 'rgb'
#==========================

rs = RSSensor()
rs.start()
while True:
    try:
        rgbd = rs.get_rgbd(workspace=workspace,depth_min=depth_min, depth_max=dept_max)
        cv2.imshow('viewer', rgbd.disp(mode=disp_mode)[:,:,::-1])
        if cv2.waitKey(10)==27:
            break
    except:
        rs.stop()