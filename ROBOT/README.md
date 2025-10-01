***
## Environment

### Linux Version : Ubuntu 20.04, 22.04(recommanded), 24.04
### Python version : 3.8
***

## 로봇 통합 SDK 환경 구성 - Local PC
### 1. Update OS
~~~
sudo apt-get update && sudo apt-get upgrade
~~~

### 2. Install dependency libraries
~~~
sudo apt-get install -y build-essential git cmake libcurl4-openssl-dev libmodbus*
sudo apt-get install -y gcc g++ gcc-multilib g++-multilib
sudo apt-get install -y python3-dev python3-pip python3-venv
pip3 install pymodbus
~~~

### 3. Downlod project
~~~
git clone https://github.com/robot-plus-program/keti-poc-talos.git
cd keti-poc-talos
git submodule update --init --recursive --remote
~~~

#### 3. Run robot & gripper integration example
```
cd ROBOT
python3 sample.py
```

## 로봇 통합 SDK 환경 구성 - Docker
### 1. 필수 패키지 설치
~~~
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
~~~

### 2. Docker 설치
~~~
# Docker GPG 키 등록
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
~~~

~~~
# Docker 저장소 추가
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
~~~

~~~
# Docker Engine 설치
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
~~~

~~~
# (선택) sudo 없이 실행하기
sudo usermod -aG docker $USER
reboot
~~~

### 3. Docker image 다운로드 및 container 생성
~~~
# Docker image 다운로드
docker pull ketiroxteam/talos-robot:latest
~~~

~~~
# Docker container 생성
docker run -it -d --network=host --name ketirobotctrl ketiroxteam/talos_robot:latest /bin/bash
~~~

~~~
# Docner container 실행
docker start -ai ketirobotctrl
~~~

### 4. 예제 코드 실행 (docker container 내부)
~~~
cd ~/project/ROBOT_SDK && python3 pythontest.py # 로봇 예제 코드
cd ~/project/GRIPPER_SDK && python3 example/example.py include lib/libzimmergripper.so # 그리퍼 예제 코드
~~~
