# HOST, PORT = 'localhost', 8808
    # INPUTS = ['rgb', 'depth', 'text']

CLIENT_TYPE = 'tcp'
HOST, PORT = '10.252.216.168', 8801
# INPUTS = ['rgb','depth','cam_params','crop_roi','min_mass','max_ratio','dmin','dmax','topn', 'nfingers']
# SAVE_DIR = None
INPUTS = ['rgb', 'caption']
SAVE_DIR = None #'/media/keti/workdir/projects/pyconnect/logs/dino'
from pyinterfaces.utils  import visualize_pred


def get_client(client_type='tcp', host='localhost', port=8888):
    if client_type=='tcp':
        from pyconnect.tcp_ip.client import TcpIpClient
        client = TcpIpClient(host=host, port=port)
    elif client_type=='zmq':
        from pyconnect.zmq.client import ZmqClient
        client = ZmqClient(host=host, port=port)
    else:
        NotImplementedError
    return client

import numpy as np
args2str = lambda inputs: ','.join([f'{k}.{v}' for k,v in inputs.items() if not isinstance(v, np.ndarray)])

def run_client(client=None):

    from pyconnect.utils import parse_keys_values, data_info, suggest_input, evaluate, resuse_recent_msg
    import cv2, os, numpy as np, glob
    from pathlib import Path

    args = parse_keys_values(optional_args={'client_type': CLIENT_TYPE,'host':HOST, 
                                            'port':PORT, 'inputs': INPUTS, 'save_dir': SAVE_DIR})
    if args['save_dir'] is not None:
        os.makedirs(args['save_dir'], exist_ok=True)
    if client is None:
        client = get_client(client_type=args['client_type'], host=args['host'], port=args['port'])
        

    while True:
        # get inputs
        inputs = {}
        for inp_name in args['inputs']:
            if inp_name in ['rgb', 'depth', 'rgbs', 'depths']:
                inputs[inp_name] = resuse_recent_msg(input(f'{inp_name}: '), recent_msg_file=inp_name)
            else:
                inputs[inp_name] = suggest_input(f'{inp_name}: ', recent_msg_file=inp_name)
            inputs[inp_name] = evaluate(inputs[inp_name])
            if inp_name in ['rgb', 'depth']:
                #
                im_dir, im_name = os.path.split(inputs[inp_name])
                im_name = os.path.splitext(im_name)[0]
                out_dir = os.path.join(im_dir, 'outputs') if SAVE_DIR is None else SAVE_DIR
                os.makedirs(out_dir, exist_ok=True)
                #
                inputs[inp_name] = cv2.imread(inputs[inp_name]) if inp_name=='rgb' else cv2.imread(inputs[inp_name], cv2.IMREAD_UNCHANGED)
                inputs[inp_name] = inputs[inp_name] if len(inputs[inp_name].shape)==1 else inputs[inp_name][...,::-1]
                
        if 'rgbs' not in inputs and 'depths' not in inputs:
            # send data and get predurn
            pred = client.send(inputs)
            print(data_info(pred))
            try:
                rgb_out = visualize_pred(rgb=inputs['rgb'], pred=pred)
                out_path  = os.path.join(out_dir, f'{im_name}_{args2str(inputs)}.png')
                cv2.imwrite(out_path, rgb_out[...,::-1])
                print(f'rgb_out saved to {out_path}')
                cv2.imshow('rgb_out', rgb_out[...,::-1])
                if cv2.waitKey()==27:
                    exit()
            except Exception as e:
                print(f'{e}. No image to show')
            
            
        else:
            rgbs = None if 'rgbs' not in inputs else glob.glob(inputs['rgbs']) 
            depths = None if 'depths' not in inputs  else glob.glob(inputs['depths']) 
            rgbs = [None]*len(depths) if rgbs is None else sorted(rgbs)
            depths = [None]*len(rgbs) if depths is None else sorted(depths)
            im_dir = os.path.split(rgbs[0])[0] if depths[0] is None else os.path.split(depths[0])[0]
            num_im = len(rgbs)
            out_dir = os.path.join(im_dir, 'outputs') if SAVE_DIR is None else SAVE_DIR
            os.makedirs(out_dir, exist_ok=True)

            print(len(rgbs), len(depths))
            assert len(rgbs)==len(depths)
            for i, (rgb_path, depth_path) in enumerate(zip(rgbs, depths)):
                print(f'[{i+1}/{num_im}]: {rgb_path} - {depth_path}')
                im_name = os.path.split(rgb_path)[-1] if depth_path is None else os.path.split(depth_path)[1]
                im_name = os.path.splitext(im_name)[0]

                inputs['rgb'] = None if rgb_path is None else cv2.imread(rgb_path)[...,::-1]
                inputs['depth'] = None if rgb_path is None else cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
                
                pred = client.send(inputs)

                #save data
                try: 
                    for i,el in enumerate(pred):
                        if isinstance(el, np.ndarray):
                            cv2.imwrite(os.path.join(out_dir, f'{im_name}_out{i}.png'), el if len(el.shape)<3 else el[...,::-1])
                except:
                    print(f'No image to save')
                    

if __name__=='__main__':
    run_client()
