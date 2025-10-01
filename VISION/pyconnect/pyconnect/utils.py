import numpy as np, pickle, sys, base64, json, os, re, cv2
from time import time
from datetime import datetime
from pathlib import Path
from prompt_toolkit import prompt

root_dir = Path(__file__).parent.parent
log_dir = os.path.join(root_dir, 'logs')
recent_msg_dir = os.path.join(log_dir, 'recent_msgs')
os.makedirs(recent_msg_dir, exist_ok=True)

# mnp.patch()

class Timer():
    def __init__(self):
        self.reset()

    def reset(self):
        self.times = [time(),]
        self.labels = []

    def __len__(self):
        return len(self.times)

    def pin_time(self, label='NaN'):
        self.add_time(label=label)
        return self.times[-1] - self.times[-2]

    def add_time(self, label='NaN'):
        self.times.append(time())
        self.labels.append(label)

    @property
    def run_time(self):
        if len(self)<2: self.add_time()
        return self.times[-1] - self.times[0]
    @property
    def fps(self):
        return 1/self.run_time

    @property
    def run_time_str(self):
        runtime=self.run_time
        if self.run_time < 1: return'Total:%dms' % (1000*runtime)
        else:  return 'Total:%.2fs' %runtime
    
    @property
    def pin_times_str(self):
        if len(self)<2: return self.run_time_str
        s = ''
        for i in range(len(self)-1):
            label  = self.labels[i]
            if label == 'NaN': continue
            dtime = self.times[i+1]-self.times[i]
            if dtime < 1: s += '%s:%dms-' %(self.labels[i], 1000*dtime)
            else: s += '%s:%.2fs-' % (self.labels[i], dtime)

        # if self.run_time() < 1:s+='Total:%dms' % (1000*self.run_time())
        # else:s+='Total:%.2fs' % self.run_time()
        s += self.run_time_str

        return s

def dict2byte(aDict):
    # return msgpack.packb(aDict, default=mnp.encode)
    return pickle.dumps(aDict)

def byte2dict(byteData):
    # return msgpack.unpackb(byteData, object_hook=mnp.decode)
    return pickle.loads(byteData)

def byte2str(byteData):
    return base64.b64encode(byteData).decode('utf-8') 

def str2byte(astr):
    return base64.b64decode(astr)

dict2str = lambda aDict: byte2str(dict2byte(aDict))
str2dict = lambda astr: byte2dict(str2byte(astr))
type_name = lambda v: type(v).__name__
strftime = lambda :datetime.now().strftime('%Y%m%d%H%M%S%f')
# pdf2text = lambda p: "\n".join(page.get_text("text") for page in fitz.open(p))

def data_info(data):
    if isinstance(data, (list, tuple)):
        return ', '.join([f'{data_info(el)}' for el in data])
    if isinstance(data, dict):
        return ', '.join([f'{k}:{data_info(v)}' for k,v in data.items()])
    if isinstance(data, np.ndarray):
        return f'array{data.shape}'
    return f'{data}'

def evaluate(v):
    if ',' not in v:
        v = v.strip()
        try: return eval(v) 
        except:  return v
    return [evaluate(el) for el in v.split(',') if len(el)>0]
    
def parse_keys_values(optional_args={}):
    
    args = sys.argv[1:] 
    args = [arg.split('=') for arg in args  if '=' in arg]
    args = {k: evaluate(v) for k, v in args}
    
    optional_args.update(args)
    print(f'{"="*10} Optional args: {", ".join([f"{k}={v}" for k,v in optional_args.items()])}')
    
    return optional_args
             
def write_json(filepath, adict):
    assert os.path.splitext(filepath)[-1]=='.json'
    with open(filepath, 'w') as f:
        json.dump(adict, f)
        
def read_json(filepath):
    assert os.path.splitext(filepath)[-1]=='.json'
    with open(filepath) as f:
        return json.load(f)

def printif(msg, do_print=True):
    if do_print:
        print(msg)



def resuse_recent_msg(msg, recent_msg_file = 'recent_msg'):
    filepath = os.path.join(recent_msg_dir, recent_msg_file)
    if len(msg)==0:
        try: 
            with open(filepath, 'r') as f:
                msg = f.read().replace('\n', '')
        except Exception as e:
            print(f'{e}')
        print(msg)
    else:
        with open(filepath, 'w') as f:
            f.write(msg)
    return msg

def findNumdersInString(sentence):
    return [eval(el) for el in re.findall(r'\d+', sentence)]

def crop_image(im, crop_roi=None, keep_size=False):
    if crop_roi is None:
        return im
    x0, y0, x1, y1 = crop_roi
    if not keep_size:
        return im[y0:y1, x0:x1,...]
    out = np.zeros_like(im)
    out[y0:y1, x0:x1,...] = im[y0:y1, x0:x1,...]
    return out

def suggest_input(guide='Input: ', recent_msg_file = 'recent_msg'):
    filepath = os.path.join(recent_msg_dir, recent_msg_file)
    try: 
        with open(filepath, 'r') as f:
            msg = f.read()
    except:
        msg = ''
    msg = prompt(guide, default=msg).replace('\\n', '\n')
    msg = resuse_recent_msg(msg, recent_msg_file=recent_msg_file)
    return msg
    
    
def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

try:
    from cv_bridge import CvBridge
except Exception as e:
    print(e)



def decode_imgmsg(msg):
    iscompressed = hasattr(msg, 'format')
    if not iscompressed:
        return {'im':CvBridge().imgmsg_to_cv2(msg, msg.encoding)}
    
    depth_fmt, compr_type = msg.format.split(';')
    depth_fmt, compr_type= depth_fmt.strip(), compr_type.strip()
    
    if 'rgb' in depth_fmt:
        return {'im':CvBridge().compressed_imgmsg_to_cv2(msg, depth_fmt)}
    
    depth_header_size = 12
    raw_data = msg.data[depth_header_size:]
    return {'im':cv2.imdecode(np.frombuffer(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)}

def encode_imgmsg(img, msg):
    encoding = 'rgb8' if len(img.shape)==3 else '16UC1'
    return CvBridge().cv2_to_imgmsg(img, encoding=encoding)


def decode_caminfomsg(msg):
    k = msg.k.reshape(3, 3)
    cam_params = [k[(0, 0)], k[(1, 1)], k[(0, 2)], k[(1, 2)]]
    return {'cam_params': cam_params}

decode_topic_strmsg = lambda msg: str2dict(msg.data)
def encode_topic_strmsg(data, msg):
    msg.data = dict2str(data)
    return msg

decode_srvserver_revmsg = lambda req: str2dict(req.req)
def encode_srvserver_retmsg(data, res):
    res.ret = dict2str(data)
    return res

decode_srvclient_revmsg = lambda res: str2dict(res.ret)
def encode_srvclient_sendmsg(data, req):
    req.req = dict2str(data)
    return req

decode_actserver_revmsg = lambda req: str2dict(req.data_goal)
def encode_actserver_retmsg(data, res):
    res.data_result = dict2str(data)
    return res

decode_actclient_recmsg = lambda res: str2dict(res.data_result)
def encode_actclient_sendmsg(data, req):
    req.data_goal = dict2str(data)
    return req

get_attrs = lambda x, atrs: {el:getattr(x, el) for el in atrs}
set_atrrs = lambda x, adict: ([setattr(x,k,v) for k,v in adict.items()], x)[-1]
update_dict = lambda adict, x: (adict.update(x), adict)[-1] 


def demo():
    # a = {'prompt': 'give me orange', 
    #      'rgb': np.random.randint(0, 255, size=(2000,2000,3), dtype='uint8'),
    #      'depth': np.random.randint(0, 65000, size=(2000,2000), dtype='uint16')}
    a = {'hello':'hello'}
    
    timer = Timer()
    b = dict2byte(a)
    timer.pin_time('dict2byte')
    c = base64.b64encode(b).decode('utf-8') 
    timer.pin_time('byte2string')
    d = base64.b64decode(c)
    timer.pin_time('string2byte')
    e = byte2dict(d)
    timer.pin_time('byte2dict')
    
    # b = dict2str(a)
    # timer.pin_time('encode')
    # c = str2dict(b)
    # timer.pin_time('decode')
    
    
    print(timer.pin_times_str)

if __name__=='__main__':
    demo()