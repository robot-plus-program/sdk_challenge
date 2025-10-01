import rclpy, numpy as np
from rclpy.action import ActionServer
from rclpy.node import Node
from  rosinterfaces.action import SendStringData as SendData
from pyconnect.utils import dict2str, str2dict, data_info

rclpy.init()
class CustomActionServer(Node):
    def __init__(self, name='action_server', action_name='send_data',run_func=None):
        super().__init__(name)
        self._action_server = ActionServer(
            self,
            SendData,
            action_name,
            self.execute_callback,
        )
        self.rev_data, self.ret_data = None, None
        self.run_func = run_func
        self.get_logger().info(f'{"="*10} Waiting for data')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal')

        data_goal = goal_handle.request.data_goal
        self.rev_data = str2dict(data_goal)
        self.get_logger().info(data_info(self.rev_data))
        

        # Process the data (here we just log it and send it back)
        feedback = SendData.Feedback()
        
        self.process_received_data()
        feedback.status = "Processing complete"
        goal_handle.publish_feedback(feedback)


        # Prepare the result
        goal_handle.succeed()
        result = SendData.Result()
        result.data_result = dict2str(self.ret_data)

        self.get_logger().info('Goal completed and data sent back')
        return result
    
    def process_received_data(self, **kwargs):
        self.ret_data = self.run_func(**self.rev_data) if self.run_func is not None else self.rev_data

    def listen(self):
        rclpy.spin(self)
        rclpy.shutdown()
        
    
        
        
def main(args=None):
    # rclpy.init(args=args)
    node = CustomActionServer()
    node.listen()

if __name__ == '__main__':
    main()
