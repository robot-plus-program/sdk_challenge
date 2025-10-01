HOST, PORT = '0.0.0.0', 8801
DETECTOR = 'grasp'#'groundedsam' 
DETECTOR_ARGS = None#{'dino_client_url': 'zmq:0.0.0.0:8801', 'fastsam_client_url':'zmq:0.0.0.0:8802'}
SERVER_TYPE = 'tcp'
import json
import argparse
import time,copy

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def get_func(detector_name, detector_args=None):
    from pyrecognition.grasp.kgrasp import talos_KGraspDetector
    from pyrecognition.instance_detection.fast_sam import InstanceDetector
    detector = talos_KGraspDetector(instance_detector=InstanceDetector())
    return detector.demo_single


def get_ang_func():
    from pyrecognition.angle.ang_detect import angle_detector
    detector = angle_detector()
    return detector.run

def get_server(server_type='tcp', host='0.0.0.0', port=8801):
    if server_type=='tcp':
        from pyconnect.tcp_ip.server import TcpIpServer
        server = TcpIpServer(host=host, port=port)
    else:
        NotImplementedError
    return server

def get_rs():
    print("get_rs")

    from ketisdk.sensor.realsense_sensor import RSSensor
    # from ketisdk.sensor.realsense_sensor import RSSensor
    import numpy as np
    sensor = RSSensor()
    sensor.get_device_sn()
    sensor.start()
    ppx, ppy, fx, fy = sensor.intr_params.ppx, sensor.intr_params.ppy, sensor.intr_params.fx, sensor.intr_params.fy
    np.save("./APP_Base/configs/intr_param.npy", [ppx, ppy, fx, fy])
    return sensor.get_data
def get_tiscamera():
    import os 
    from tis.tiscamera import TISCamera

    setting_path=__file__.split("run_server_talos")[0]
    setting_filepath=f'{setting_path}/APP_Base/configs/tis_camera.json'
    with open(setting_filepath, "r", encoding="utf-8") as f:
        tis_setting = json.load(f)
    print(tis_setting)
    tisCam = TISCamera()
    tisCam.set_device(tis_setting['device_serial'], tis_setting['sensor_size'], tis_setting['fps'], showvideo=False)
    tisCam.start()

    
    # 초기 카메라 파라미터 수동 설정
    FocusAuto = tis_setting["FocusAuto"]
    Focus = tis_setting["Focus"]

    GainAuto = tis_setting["GainAuto"]
    Gain = tis_setting["Gain"]

    ExposureTime = tis_setting["ExposureTime"]
    ExposureAuto = tis_setting["ExposureAuto"]
    

    tisCam.Tis.Set_Property("FocusAuto", FocusAuto)
    tisCam.Tis.Set_Property("Focus", Focus)

    tisCam.Tis.Set_Property("GainAuto", GainAuto)
    tisCam.Tis.Set_Property("Gain", Gain)

    tisCam.Tis.Set_Property("ExposureAuto", ExposureAuto)
    tisCam.Tis.Set_Property("ExposureTime", ExposureTime)


    pre_exp=tisCam.Tis.Get_Property("ExposureTime")
    count=0
    while 1:
        cur_exp = tisCam.Tis.Get_Property("ExposureTime")
        print(cur_exp,end=" ")
        time.sleep(0.1)
        if cur_exp==pre_exp:
            count+=1
        pre_exp=copy.deepcopy(cur_exp)
        if count>20:
            break
    print("done")
    return tisCam.get_image

def run_server(custom_func=None, custom_func_name='custom_func', server=None, server_type='custom_sever',
               flag_rs=True,flag_tis=True):
    from pyrecognition.utils import parse_keys_values
    args = parse_keys_values(optional_args={ 'server_type': SERVER_TYPE if server is None else server_type,
        'host': HOST, 'port': PORT, 'detector_args': DETECTOR_ARGS,
        'detector': DETECTOR if custom_func is None else custom_func_name})

    detector_name = args['detector'].strip().lower()
    run_func =  get_func(detector_name=detector_name, detector_args=args['detector_args']) if custom_func is None else custom_func
    run_ang_func=get_ang_func()
    server = get_server(server_type=args['server_type'], host=args['host'], port=args['port'])

    server.run_func = run_func
    server.run_ang_func=run_ang_func
    if flag_tis:
        print(f"TIS RGB 카메라 연결 (flag_tis={flag_tis})")
        server.run_func_tisCam=get_tiscamera()

    else:
        print(f"TIS 카메라 연결되지 않음 (flag_tis={flag_tis})")
    if flag_rs:
        print(f"Realsense 카메라 연결 (flag_rs={flag_rs})")
        server.run_func_rs=get_rs()
    else:
        print(f"Realsense 연결되지 않음 (flag_rs={flag_rs})")
    server.listen()

if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--rs', type=str2bool, default=True, help='connect realsense camera')
    parser.add_argument('--rgb', type=str2bool, default=True, help='connect tis RGB camera')
    args = parser.parse_args()
    print(args)
    print("args.rs",args.rs)
    print("args.rgb",args.rgb)
    run_server(flag_rs=args.rs,flag_tis=args.rgb)
