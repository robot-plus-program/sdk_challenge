from ketisdk.sensor.realsense_sensor import RSSensor
from ketisdk.base.base_tcp import ClientThread, ZeromqClient
import cv2

def get_args():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost', type=str, help='Host IP address')
    parser.add_argument('--port', default=8888, type=int, help='Port address')
    parser.add_argument('--depth_min', default=500, type=int, help='Min value of depth sensor')
    parser.add_argument('--dept_max', default=700, type=int, help='Max value of depth sensor')
    parser.add_argument('--disp_mode', default='rgb', type=str, help='Display mode')
    parser.add_argument('--send_text', default=False, type=bool, help='Send text or not')
    args = parser.parse_args()
    return args
def main(args):
    #========================== configs
    workspace=None
    #==========================
    client = ClientThread(host=args.host, port=args.port)
    # client = ZeromqClient(host=args.host, port=args.port)

    rs = RSSensor()
    rs.start()
    while True:
        rgbd = rs.get_rgbd(workspace=workspace, depth_min=args.depth_min, depth_max=args.dept_max)
        cv2.imshow('viewer', rgbd.disp(mode=args.disp_mode)[:, :, ::-1])
        key = cv2.waitKey(10)
        if key == 27:
            break
        elif key == 32:  # space
            data = {'rgb': rgbd.rgb, 'depth': rgbd.depth }
            if args.send_text:
                text = input('Text to be sent? ')
                data['text'] = text
            ret = client.send_and_get_return(data)
            if ret is None:
                print('None return .....')
                continue
            cv2.imshow('viewer', ret['im'][:, :, ::-1])
            cv2.waitKey()

    rs.stop()

if __name__=='__main__':
    main(get_args())