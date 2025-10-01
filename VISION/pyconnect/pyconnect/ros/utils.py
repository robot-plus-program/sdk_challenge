import os, glob, time, numpy as np, cv2, threading, copy
from pyconnect.utils import dict2str, str2dict, strftime, write_json, data_info, read_json
from std_msgs.msg import String
from pathlib import Path
root_dir = Path(__file__).parent.parent.parent

log_node_dir = os.path.join(root_dir, 'logs/ros/node')
log_sub_dir = os.path.join(root_dir, 'logs/ros/sub')
os.makedirs(log_node_dir, exist_ok=True)
os.makedirs(log_sub_dir, exist_ok=True)


def find_last_session_log_dir(log_dir=log_node_dir):
    dirs = [el for el in glob.glob(os.path.join(log_dir, '*')) if os.path.isdir(el)]
    if len(dirs)==0:
        return None
    return sorted(dirs)[-1]


def default_strmsg_revfunc(msg):
    return str2dict(msg.data)

def default_strmsg_resfunc(data):
    msg = String()
    msg.data = dict2str(data)
    return msg

def default_srvstr_revfunc(req, is_server=True):
    return str2dict(req.req if is_server else req.ret)

def default_srvstr_resfunc(data, res, is_server=True):
    if is_server:
        res.res = dict2str(data)
    else:
        res.ret = dict2str(data)
    return res

def node_add_agent_from_config(node, config, agent_name=None, mode=''):  
    if config['agent_type']=='service_server':
        callback_group = node.callback_groups[-1]
    elif config['agent_type']=='sub':
        callback_group = node.callback_groups[-2]
    else:
        callback_group = None
    
    if 'agent_name' not in config:
        config.update({'agent_name': agent_name})
    config['agent_name'] = f'{mode}{config["agent_name"]}'

    node.add_agent(callback_group=callback_group, **config)
    return node


def node_add_agents_from_configs(node, configs, agents_select=None, mode=''):   
    
    for agent_name, cfg in configs.items():
        if agents_select is not None:
            if agent_name not in agents_select:
                continue

        cfg = configs[agent_name]
        
        node = node_add_agent_from_config(node=node, agent_name=agent_name, config=cfg, mode=mode)

    return node



class SubLog():
    def __init__(self, node, every=1.):
        self.log_dir = None
        self.do_log = False
        self.every, self.node = every, node
        threading.Thread(target=self.log, daemon=True).start()
    
    def start(self, log_dir=''):
        self.log_dir =  os.path.join(log_sub_dir, strftime() + log_dir)
        self.do_log = True
        print(f'log all sub received data every {self.every}s to {self.log_dir}')

    def stop(self):
        self.do_log = False


    def log(self):
        while True:
            if self.do_log:
                sub_dir = os.path.join(self.log_dir, strftime())
                sub_im_dir = os.path.join(sub_dir, 'array')
                os.makedirs(sub_dir, exist_ok=True)
                os.makedirs(sub_im_dir, exist_ok=True)
                for agent_name, agent in self.node.agents.items():
                    if agent.type!='sub':
                        continue
                    data = agent.rev_data
                    if data is None:
                        continue

                    agent_name = agent_name.replace('/','.')
                    for k,v in data.items():

                        if isinstance(v, np.ndarray):
                            im_name = f'{agent_name}_{k}.png' if 'im' in k else f'{agent_name}_{k}.npy'
                            data[k] = im_name

                            im_path = os.path.join(sub_im_dir, im_name)
                            cv2.imwrite(im_path, v if len(v.shape)<3 else v[...,::-1]) if 'im' in k else np.save(im_path, v)
                                
                    write_json(os.path.join(sub_dir, f'{agent_name}.json'), data)
            time.sleep(self.every)


class ShowRosCam:
    def __init__(self, agent):
        self.agent = agent
        self.do_show = False
        threading.Thread(target=self.show, daemon=True).start()
    
    def start(self):
        self.do_show = True

    def stop(self):
        self.do_show  = False

    def show(self):
        while True:
            if self.do_show:
                try:
                    cv2.imshow(self.agent.name, self.agent.rev_data['im'][...,::-1])
                    cv2.waitKey(30)
                except Exception as e:
                    print(f'Show Ros Cam Error: {e}')
                    time.sleep(0.03)
            else:
                time.sleep(0.03)

def get_states_from_trajectory(trajectory=None, log_dir=log_sub_dir):
    if trajectory is None or len(trajectory)==0:
        trajectory_dir = find_last_session_log_dir(log_dir=log_dir)
    else:
        trajectory_dir = os.path.join(log_dir, trajectory)
    
    if not os.path.exists(trajectory_dir):
        print(f'{trajectory_dir} does not exist')
        return []
        
    
    action_dirs = sorted([el for el in glob.glob(os.path.join(trajectory_dir, '*')) if os.path.isdir(el)])
    
    trajectory_name = os.path.split(trajectory_dir)[-1]
    prompt = trajectory_name.split('@')[-1] if '@' in trajectory_name else '' 
    states = [{'prompt': prompt.replace('_', ' ')}]
    for action_dir in action_dirs:
        action_states = dict()
        for json_path in glob.glob(os.path.join(action_dir, '*.json')) + glob.glob(os.path.join(action_dir, '.*.json')):
            agent_name = os.path.splitext(os.path.split(json_path)[-1])[0].replace('.','/')
            data = read_json(json_path)
            for k,v in data.items():
                if not isinstance(v, str):
                    continue
                p = os.path.join(action_dir, 'array', v)
                if not os.path.exists(p):
                    continue
                # v = np.load(p) if '.npy' in p else cv2.imread(p, cv2.IMREAD_UNCHANGED) if 'depth' in p else cv2.imread(p)[...,::-1]
                data[k] = p
            
            action_states.update({agent_name: data})
        
        states.append(action_states)
    
    return states

        
if __name__=='__main__':
    get_states_from_trajectory()


        

        
                
                    
            
        

    



    