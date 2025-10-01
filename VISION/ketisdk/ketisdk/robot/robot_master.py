"""
    KETI ROBOT_MASTER
    ~~~~~~~~~~~~~~~~~
"""
from abc import *

class Robot(object):
    """ROBOT 운영을 위한 기본 클래스.

       :param ip: 서버 ip
       :param port: 서버 port
       :param proj: project 종류 (p0:FPCB, p1:무진, p2:협동로봇, ...)
       :param workspace: workspace 종류 (w0:실험실, w1:현장1, ...)
       :type ip: str
       :type port: int
       :type proj: str
       :type workspace: str

       예제:
            다음과 같이 사용하세요:

             >>> robot = Hcr3('192.168.100.1',6666,'p0','ws0')


    """
    robot_state = None
    ip = None
    port = None
    proj = None
    workspace = None
    c_pose = [0,0,0,0,0,0]
    c_joint = [0,0,0,0,0,0]
    c_jspeed = [0,0,0,0,0,0]

    def __init__(self, ip, port, proj, workspace):
        print('Robot::init')
        self.ip = ip
        self.port = port
        self.proj = proj
        self.workspace = workspace
        self.load_params()
        self.connect_robot()

    def __del__(self):
        self.disconnect_robot()
        print('Robot::closed')

    @abstractmethod
    def load_params(self):
        """ Robot parameters load
        """
        #print('Robot::Not implemented load_params in robot!!!!')
        pass

    @abstractmethod
    def reload_params(self):
        """ Robot parameters reload
        """
        #print('Robot::Not implemented reload_params in robot!!!!')
        pass

    @abstractmethod
    def save_params(self):
        """ Robot parameters save
        """
        print('Robot::Not implemented save_params in robot!!!!')

    @abstractmethod
    def connect_robot(self):
        """ Robot 네트워크 연결
        """
        #print('Robot::Not implemented connect_robot in robot!!!!')
        pass

    @abstractmethod
    def disconnect_robot(self):
        """ Robot 네트워크 해제
        """
        #print('Robot::Not implemented disconnect_robot in robot!!!!')
        pass

    @abstractmethod
    def move_j(self, joint_param, vel, acc, time, move_wait = True):
        
        """ Robot joint 이동

            :param joint_param: joint_param (float형 6개)
            :param vel: Robot Move 속도
            :param acc: Robot Move 가속도
            :param time: Robot Move 시간
            :param move_wait: ROBOT 도달 보장
            :type joint_param: list
            :type vel: float
            :type acc: float
            :type time: float
            :type move_wait: bool

            예제:
                다음과 같이 사용하세요:

                >>> robot.move_j([j1,j2,j3,j4,j5,j6],800,1200,0.1)
        """
        pass

    @abstractmethod
    def move_l(self, pose_param, vel, acc, time, move_wait = True):

        """ Robot 직선 이동

                :param pose_param: pos_param (float형 6개)
                :param vel: Robot Move 속도
                :param acc: Robot Move 가속도
                :param time: Robot Move 시간
                :param move_wait: ROBOT 도달 보장
                :type pose_param: list
                :type vel: float
                :type acc: float
                :type time: float
                :type move_wait: bool

                예제:
                    다음과 같이 사용하세요:

                    >>> robot.move_l([x,y,z,lx,ly,lz],800,1200,0.1)

        """
        pass
        #print('Robot::Not implemented move_l in robot!!!!')

    @abstractmethod
    def set_home_rot(self):
        """
            Robot 초기 세팅 위치 설정.

             :param pos_param: pos_param (float형 6개)
             :type pos_param: list
         """

        print('Robot::Not implemented set_home_rot in robot!!!!')

    @abstractmethod
    def move_home(self):
        """
             Home 위치로 로봇 이동
        """
        #print('Robot::Not implemented move_home in robot!!!!')

    @abstractmethod
    def set_init_pos(self, pos_param):
        """
        Robot 프로그램 초기 위치 설정.

        :param pos_param: pos_param (float형 6개)
        :type pos_param: list
        """
        #print('Robot::Not implemented set_init_pos in robot!!!!')

    @abstractmethod
    def move_init(self):
        """
        Init 위치로 로봇 이동
        """
        #print('Robot::Not implemented move_init in robot!!!!')

    @abstractmethod
    def set_drop_pos(self):
        """
               Robot 프로그램 작업 후 위치 설정.

               :param pos_param: pos_param (float형 6개)
               :type pos_param: list
        """
        #print('Robot::Not implemented set_drop_pos in robot!!!!')

    @abstractmethod
    def move_drop(self):
        """
            작업 후 위치로 로봇 이동
        """
        #print('Robot::Not implemented move_drop in robot!!!!')

    @abstractmethod
    def stop(self):
        """
            로봇 스탑 함수
        """
        print('Robot::Not implemented stop in robot!!!!')

    @abstractmethod
    def get_pos(self):
        """
            Robot 현재 pos 값 반환

            :return: pos_param: pos_param
            :rtype: pos_param: list
         """

    @abstractmethod
    def get_rot(self):
        """
            Robot 현재 pos 값 반환

            :return: pos_param: pos_param
            :rtype: list (float형 6개)
        """
        #print('Robot::Not implemented get_rot in robot!!!!')

    @abstractmethod
    def turn_on_io(self, ioNum):
        """
            Robot io 전원 ON

            :param ioNum: Robot IO
            :type ioNum: int
        """
        print('Robot::Not implemented turn_on_io in robot!!!!')

    @abstractmethod
    def turn_off_io(self, ioNum):
        """
            Robot io 전원 OFF

            :param ioNum: Robot IO
            :type ioNum: int
        """
        print('Robot::Not implemented turn_off_io in robot!!!!')


if __name__ == "__main__":
    import doctest
    doctest.testmod()