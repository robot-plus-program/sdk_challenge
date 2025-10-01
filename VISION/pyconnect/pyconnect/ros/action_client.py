import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosinterfaces.action import SendStringData as SendData  
import numpy as np
from pyconnect.utils import dict2str, str2dict, data_info

rclpy.init()

class CustomActionClient(Node):
    def __init__(self, name='action_client', action_name='send_data'):
        super().__init__(name)
        self._action_client = ActionClient(self, SendData, action_name)
        self.rev_data=None

    def send(self, data):
        goal_msg = SendData.Goal()
        data_goal = dict2str(data)
        goal_msg.data_goal = data_goal
        # for k,v in data.items():
        #     self.get_logger().info(f'{k}: {v.shape if isinstance(v,np.ndarray) else v}')

        self.get_logger().info('Sending goal to server...')
        self._action_client.wait_for_server()
        self._goal_handle = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._goal_handle.add_done_callback(self.goal_response_callback)
        
        rclpy.spin(self)
        

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.status}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.rev_data = str2dict(result.data_result)
        self.get_logger().info(data_info(self.rev_data))
        rclpy.shutdown()
        

def main(args=None):
    # rclpy.init(args=args)
    client = CustomActionClient()
    client.send({'text': 'hello', 'rgb': np.random.randint(0,255, size=(2000, 2000,3), dtype='uint8'),
                       'depth': np.random.randint(0, 65000, size=(2000, 2000), dtype='uint16')})

if __name__ == '__main__':
    main()
