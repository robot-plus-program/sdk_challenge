***
## Environment

### Linux Version : Ubuntu 20.04
***

<br>

## 1. 운영체제 업데이트
~~~
sudo apt-get update && sudo apt-get upgrade
~~~

## 2. library 설치
~~~
sudo apt-get install cmake build-essential git 
sudo apt-get install libmodbus*
~~~

## 2. KETI SDK 다운로드
~~~
git clone https://github.com/robot-plus-program/sdk_challenge.git
cd sdk_challenge
git clone https://github.com/robot-plus-program/ketirobotsdk.git
~~~

## 3. Build & Run example
### 3.1 C++
~~~
cd ${download sdk_challenge folder}
mkdir build
cd build
cmake ..
make
./example xxx.xxx.xxx.xxx ooo.ooo.ooo.ooo port_num
~~~
### 3.2 Python
~~~
cd ${download sdk_challenge folder}/scripts
chmod +x example.py
python3 example.py xxx.xxx.xxx.xxx ooo.ooo.ooo.ooo port_num
~~~
xxx.xxx.xxx.xxx : Robot IP address<br>
ooo.ooo.ooo.ooo : Gripper IP address<br>
port_num : Gripper port number<br>

***
## 4. Simulator
### 4.1 ROS noetic 설치<br>
http://wiki.ros.org/noetic/Installation/Ubuntu<br>
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment<br>

### 4.2 ROS moveit 설치
~~~
sudo apt-get install ros-noetic-moveit*
~~~

### 4.3 ROS controller 설치
~~~
sudo apt-get install ros-noetic-controller-*
sudo apt-get install ros-noetic-joint-trajectory-controller
~~~

### 4.4 library 설치
~~~
sudo apt-get install cmake build-essential git 
~~~
\# SDK를 실행하는 PC와 시뮬레이터 PC가 동일 할 경우 한 번만 설치하면 됨.

### 4.5 ROS 환경 설정
터미널 접속 후 **gedit ~/.bashrc** 명령어 실행, 아래 내용 추가  
~~~
source ~/catkin_ws/devel/setup.bash
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
~~~
bashrc 편집 후 터미널에서 **source ~/.bashrc** 명령어 실행  
\# **~/catkin_ws**는 ROS 설치 시 설정에 따라 변경 될 수 있음  
ex<br>
![ros_environment_example](./imgs/ros_environment_example.png)

### 4.6 ROS workspace로 source code 복사 및 빌드
~~~
cd ${download sdk_challenge folder}
cp -r keti_sdk_simulator/ ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make
~~~

### 4.5 Simulator 실행
~~~
roslaunch keti_robot_control robot_control.launch
~~~
![simulator_run](./imgs/simulator_run.png)
