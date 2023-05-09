***
## Environment

### Linux Version : Ubuntu 20.04
***

<br>

## 1. 운영체제 업데이트
~~~
sudo apt-get update && sudo apt-get upgrade
~~~

## 2. Modbus library 설치
~~~
sudo apt-get install libmodbus*
~~~

## 2. KETI SDK 다운로드
~~~
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
http://wiki.ros.org/noetic/Installation/Ubuntu 참고
<br>

### 4.2 ROS moveit 설치
~~~
sudo apt-get install ros-noetic-moveit*
~~~

### 4.3 Modbus library 설치
~~~
sudo apt-get install libmodbus*
~~~
\* SDK를 실행하는 PC와 시뮬레이터 PC가 동일 할 경우 한 번만 설치하면 됨.

### 4.4 ROS workspace로 source code 복사 및 빌드
~~~
cd ${download sdk_challenge folder}
cp -r keti_sdk_simulator/ ~/{ros workspace}/src/
cd ~/{ros workspace}/
catkin_make
~~~

### 4.5 Simulator 실행
~~~
roslaunch keti_robot_control robot_control.launch
~~~