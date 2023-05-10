***
## Environment

### Linux Version : Ubuntu 20.04
***

<br>

## 1. 운영체제 업데이트 및 
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
sudo apt-get install git
git clone https://github.com/robot-plus-program/sdk_challenge.git
~~~

## 3. Build & Run example
~~~
cd ${download sdk_challenge folder}
mkdir build
cd build
cmake ..
make
./example xxx.xxx.xxx.xxx ooo.ooo.ooo.ooo port_num
~~~
xxx.xxx.xxx.xxx : Robot IP address<br>
ooo.ooo.ooo.ooo : Gripper IP address<br>
port_num : Gripper port number
<br><br>

***
## 4. Simulator
### 4.1 ROS noetic 설치<br>
http://wiki.ros.org/noetic/Installation/Ubuntu
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment 참고
<br>

### 4.2 ROS moveit 설치
~~~
sudo apt-get install ros-noetic-moveit*
~~~

### 4.3 library 설치
~~~
sudo apt-get install cmake build-essential git 
sudo apt-get install libmodbus*
~~~
\* SDK를 실행하는 PC와 시뮬레이터 PC가 동일 할 경우 한 번만 설치하면 됨.

### 4.4 ROS 환경 설정
터미널 접속 후 gedit ~/.bashrc 명령어 실행, 아래 내용 추가
~~~
source ${ros workspace}/devel/setup.bash
alias cw='cd ${ros workspace}'
alias cs='cd ${ros workspace}/src'
alias cm='cd ${ros workspace} && catkin_make'
~~~
bashrc 편집 후 터미널에서 source ~/.bashrc 명령어 실행

### 4.4 ROS workspace로 source code 복사 및 빌드
~~~
cd ${download sdk_challenge folder}
cp -r keti_sdk_simulator/ ${ros workspace}/src/
cd ${ros workspace}/
catkin_make
~~~

### 4.5 Simulator 실행
~~~
roslaunch keti_robot_control robot_control.launch
~~~