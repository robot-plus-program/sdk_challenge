from abc import  abstractmethod
import rclpy, threading, os, numpy as np, cv2, glob, time, threading
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from cv_bridge import CvBridge
# from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from pyconnect.utils import  str2dict, dict2str, data_info, parse_keys_values, strftime, write_json, printif
from pyconnect.ros.utils import find_last_session_log_dir, log_node_dir
try:
    from rosinterfaces.action import SendStringData as SendData
    from rosinterfaces.srv import SendStringData as SendSeviceData
except:
    printif('No SendStringData defined.')
from pathlib import Path


rclpy.init()

class NodeAgent:
    def __init__(self, name='unnamed_agent', atype='sub', executor=None, is_init=False, data_interface=String,
                 encode_func=None, decode_func=None, response_func=None, do_log_msg=False,**kwargs):
        self.name, self.type, self.executor = name, atype, executor
        self.rev_data, self.ret_data, self.msg_id = None, None, None
        self.id = f'{self.name}.{self.type}'
        self.is_init, self.do_log_msg = is_init, do_log_msg        
        self.data_interface = data_interface
        self.encode_func, self.decode_func, self.response_func = encode_func, decode_func, response_func
        self.connected = False
    
    def make_log_dirs(self):
        session_dir = find_last_session_log_dir()
        if self.is_init or session_dir==None:
            session_dir = os.path.join(log_node_dir, strftime())
        self.log_msg_dir = os.path.join(session_dir, 'msg')
        self.log_img_dir = os.path.join(session_dir, 'img')
        os.makedirs(self.log_msg_dir, exist_ok=True)
        os.makedirs(self.log_img_dir, exist_ok=True)

    def log_msg(self, data, msg_type):
        self.make_log_dirs()
        msg_id = f'{strftime()}@{self.id}@{msg_type}'.replace('/','.')
        try:
            # not ndarray
            write_json(os.path.join(self.log_msg_dir, f'{msg_id}.json'),
                    adict={k:v for k,v in data.items() if not isinstance(v, np.ndarray) and k!='ins'} )
            # array
            for k, v in  data.items():
                if isinstance(v, np.ndarray):
                    try:
                        cv2.imwrite(os.path.join(self.log_img_dir, f'{msg_id}_{k}.png'), v if len(v.shape)<=2 else v[...,::-1])
                    except:
                        np.save(os.path.join(self.log_img_dir, f'{msg_id}_{k}.npy'), v)
                elif k=='ins':
                    np.save(os.path.join(self.log_img_dir, f'{msg_id}_{k}.npy'), v)
        except:
            np.save(os.path.join(self.log_msg_dir, f'{msg_id}.npy'), data)
        printif(f'{msg_id} saved', do_print=self.do_log_msg)   
        
    def reform_response(self,ret_data):
        if not isinstance(ret_data, dict):
            return {'isdone': True, 'ret': ret_data}
        if 'isdone' in ret_data:
            return  ret_data
        ret_data['isdone'] = True
        return ret_data
        
class ActionSeverAgent(NodeAgent):
    def callback(self, goal_handle):
        printif(f'{"="*5}{self.id}: Received goal', do_print=self.do_log_msg)
        assert self.encode_func is not None and self.decode_func is not None

        request = goal_handle.request
        rev_data = self.decode_func(request)
        
        # 
        feedback = self.data_interface.Feedback()
        ret_data = rev_data if self.response_func is None else self.response_func(rev_data)
        ret_data = self.reform_response(ret_data)
        
        feedback.status = "Processing complete"
        goal_handle.publish_feedback(feedback)


        # Prepare the result
        goal_handle.succeed()
        result = self.data_interface.Result()
        result = self.encode_func(ret_data, result)

        printif(f'{"="*5}{self.id}: Goal completed and data sent back', do_print=self.do_log_msg)
        return result
    
class ActionClientAgent(NodeAgent):    
        
    def send(self, data):
        assert self.encode_func is not None and self.decode_func is not None
        if self.do_log_msg:
            printif(data_info(data), do_print=self.do_log_msg)
            self.log_msg(data, msg_type='sent')

        goal_msg = self.data_interface.Goal()
        goal_msg = self.encode_func(data, goal_msg)

        printif(f'{"="*5}{self.id}: Sending goal to server...', do_print=self.do_log_msg)
        if not self.executor.wait_for_server(timeout_sec=3.0):
            printif(f'{"="*5}{self.id} Action Sever not available', do_print=self.do_log_msg)
        
        goal_handle = self.executor.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        goal_handle.add_done_callback(self.goal_response_callback)    
        

    def feedback_callback(self, feedback_msg):
        printif(f'{"="*5}{self.id} Feedback: {feedback_msg.feedback.status}', do_print=self.do_log_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            printif(f'{"="*5}{self.id}: Goal was rejected', do_print=self.do_log_msg)
            return

        printif(f'{"="*5}{self.id}: Goal accepted, waiting for result...', do_print=self.do_log_msg)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.rev_data = self.decode_func(result)

        printif(f'{"="*5}{self.id}: Return received...', do_print=self.do_log_msg)
        if self.do_log_msg:
            printif(data_info(self.rev_data), do_print=self.do_log_msg)
            self.log_msg(self.rev_data, msg_type='retunred')
        
class TopicAgent(NodeAgent):
    
    def callback(self, msg):
        
        if not hasattr(self, 'is_firstmsg'):
            self.is_firstmsg = True

        assert self.decode_func is not None
        data = self.decode_func(msg)
        if self.is_firstmsg:
            print(f'{"="*5}{self.id}: msg received')
            print(data_info(data))
            self.is_firstmsg = False
        self.rev_data = data
        
        if self.response_func is not None:
            self.response_func(data)
            print(f'{"="*5}{self.id}: Process finished')
            
        
    def send(self, data, do_log_msg=False):
        assert self.encode_func is not None


        # if not hasattr(self, 'is_firstmsg'):
        #     self.is_firstmsg = True
        if self.do_log_msg:
            print(f'{"="*5}{self.id}: Data pulished')
            self.log_msg(data=data, msg_type='sent')
            

        msg = self.data_interface()
        msg = self.encode_func(data, msg)
        self.executor.publish(msg)

    def publish(self):
        assert self.encode_func is not None

        msg = self.data_interface()
        msg = self.encode_func(None, msg)

        if not hasattr(self, 'is_firstmsg'):
            self.is_firstmsg = True
        if self.is_firstmsg:
            printif(f'{"="*5}{self.id}: Data pulished', do_print=self.do_log_msg)
            self.is_firstmsg = False

        self.executor.publish(msg)


class ServiceServerAgent(NodeAgent):

    def callback(self, request, response):
        print(f'{"="*5}{self.id}: Data received')
        assert self.decode_func is not None and self.encode_func is not None

        rev_data = self.decode_func(request)
        print(f'{"="*5}{self.id}: Processing')
        ret_data = rev_data if self.response_func is None else self.response_func(rev_data)
        ret_data = self.reform_response(ret_data)

        response = self.encode_func(ret_data, response)
        print(f'{"="*5}{self.id}: Response sent...')
        return response

class ServiceClientAgent(NodeAgent):
        
    
    def send(self, data, wait_until_done=True):
        assert self.encode_func is not None and self.decode_func is not None
        if self.do_log_msg:
            printif(data_info(data), do_print=self.do_log_msg)
            self.log_msg(data, msg_type='sent')

        printif(f'{"="*5}{self.id}: Data sent', do_print=self.do_log_msg)
        try:
            self.request = self.encode_func(data, self.request)
        except Exception as e:
            print(f'{"="*5}{self.id}: Error: {e}. Stopped ...')
            return {'isdone': False} 

        future = self.executor.call_async(self.request)
        while rclpy.ok() and not future.done() and wait_until_done:
            time.sleep(0.1)

        result = future.result()
        ret_data = self.decode_func(result)
        printif(f'{"="*5}{self.id}: Return received', do_print=self.do_log_msg)
        if self.do_log_msg:
            printif(data_info(ret_data), do_print=self.do_log_msg)
            self.log_msg(ret_data, msg_type='returned')
        return ret_data


AGENT_CLASS_DICT={
    'action_server': ActionSeverAgent,
    'action_client': ActionClientAgent,
    'sub': TopicAgent,
    'pub': TopicAgent,
    'service_server': ServiceServerAgent,
    'service_client': ServiceClientAgent
}



class CustomNode(Node):
    
    def __init__(self, name='unnamed_node', is_init_node=False, num_callbackgroup=0, **kwargs):
        super().__init__(name)
        self.name, self.is_init, self.num_callbackgroup = name, is_init_node, num_callbackgroup
        self.agents = dict()
        if num_callbackgroup>0:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            self.callback_groups = [MutuallyExclusiveCallbackGroup() for _ in range(num_callbackgroup)]
        self.get_logger().info(f'{"="*10} Waiting for data')
        self._logger
        
        
    def add_agent(self, agent_name='unnamed_agent', agent_type='sub', data_interface=SendData, callback_group=None,
                  encode_func=None, decode_func=None, response_func=None, do_log_msg=False, **kwargs):
        
        agent = AGENT_CLASS_DICT[agent_type](name=agent_name, atype=agent_type, is_init=self.is_init, data_interface=data_interface, 
                                             encode_func=encode_func, decode_func=decode_func, response_func=response_func, do_log_msg=do_log_msg)
        self.is_init = False
        if agent_type=='action_server':
            ActionServer(self, data_interface, agent_name, agent.callback, callback_group=callback_group)
        elif agent_type=='action_client':
            executor=ActionClient(self, data_interface, agent_name)
            agent.executor = executor
        elif agent_type=='sub':
            self.create_subscription(data_interface, agent_name, agent.callback, 10, callback_group=callback_group)
        elif agent_type=='pub':
            executor = self.create_publisher(data_interface, agent_name, 10)
            agent.executor = executor
            if 'time_period' in kwargs:
                self.create_timer(kwargs['time_period'], agent.publish)
        elif agent_type=='service_server':
            self.create_service(data_interface,  agent_name, agent.callback, callback_group=callback_group)
        elif agent_type=='service_client':
            executor = self.create_client(data_interface, agent_name)
            agent.executor = executor
            def wait_service():
                print(f'Waiting for {agent_name} service...')
                while not executor.wait_for_service(timeout_sec=1.0):
                    # print(f'Waiting for {agent_name} service...')
                    pass
                print(f'Service {agent_name} connected ...')
                agent.connected = True
                agent.request = data_interface.Request()
            threading.Thread(target=wait_service, daemon=True).start()
        else:
            NotImplementedError
            
        self.agents[agent_name]=agent
        print(f'===== {agent_name} {agent_type} added to node ...')
        
            
    def listen(self, run_thread=False):
        def listen_():
            try:
                if self.num_callbackgroup==0:
                    rclpy.spin(self)
                else:
                    executor = MultiThreadedExecutor(num_threads=self.num_callbackgroup)
                    executor.add_node(self)
                    executor.spin()
            except KeyboardInterrupt:
                pass
            finally:
                self.destroy_node()
                rclpy.shutdown()

        if run_thread:
            thread = threading.Thread(target=listen_, daemon=True)
            thread.start()
        else:
            listen_()
    
    def spin(self, run_thread=False):
        self. listen(run_thread=run_thread)

def test_action_server():        

    # server node
    server_node = CustomNode(name='server', num_callbackgroup=2)
    server_node.add_agent(agent_name='send_data', agent_type='action_server', data_interface=SendData, callback_group=server_node.callback_groups[-1])
    
    def decode_func(req):
        return str2dict(req.data_goal)
    
    def encode_func(data, res):
        res.data_result = dict2str(data)
        return res

    server_node.agents['send_data'].encode_func = encode_func
    server_node.agents['send_data'].decode_func = decode_func
    server_node.spin(run_thread=True)

    # client node
    client_node = CustomNode(name='client')
    client_node.add_agent(agent_name='send_data', agent_type='action_client', data_interface=SendData)

    def decode_func(res):
        return str2dict(res.data_result)
    
    def encode_func(data, req):
        req.data_goal = dict2str(data)
        return req

    client_node.agents['send_data'].encode_func = encode_func
    client_node.agents['send_data'].decode_func = decode_func
    client_node.spin(run_thread=True)
    
    

    while True:
        client_node.agents['send_data'].send({'prompt': 'hello'})
        rev_data = client_node.agents['send_data'].rev_data
        time.sleep(2)

def test_topic():
    from pyconnect.utils import decode_topic_strmsg, set_atrrs, dict2str
    node = CustomNode(name = 'server')
    node.add_agent(agent_name='send_data', agent_type='pub', data_interface=String, 
                   encode_func=lambda data, msg: set_atrrs(msg, {'data': dict2str({'prompt': 'hello'})}),
                   do_log_msg=False, time_period=0.5)
    node.spin(run_thread=True)

    node = CustomNode(name='client', num_callbackgroup=2)
    node.add_agent(agent_name='send_data', agent_type='sub', data_interface=String, 
                   decode_func=decode_topic_strmsg, do_log_msg=True, 
                   callback_group=node.callback_groups[-1])
    node.spin()
    
def test():
    node = CustomNode()
    node.add_agent(agent_type='pub', agent_name='send_text')
    node.listen_thread()
    while True:
        msg = input("Enter message to publish (or 'exit' to quit): ")
        if msg.lower() == 'exit':
            break
    
        node.agents['send_text'].send(msg)
            
            
if __name__=='__main__':
    test_topic()