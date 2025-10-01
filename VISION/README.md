# 00. Environment

- Prerequisites: 
  - Ubuntu version 20.04 or later (20.04 recommended)
  - Python 3.8 
  - Cuda 11.3 or later
- Dependencies Installation
  ~~~
  sudo apt update

  python3 -V
  
  # python3.8
  sudo apt install software-properties-common
  sudo add-apt-repository ppa:deadsnakes/ppa
  sudo apt-cache policy python3.8
  sudo apt install python3.8
  
  sudo apt update
  sudo apt install terminator htop  -y
  sudo apt install python3.8-dev  python3.8-venv python3.8-tk python3-pip -y
  sudo apt install build-essential cmake pkg-config
  sudo apt install libffi-dev libssl-dev
  sudo apt install libxml2-dev libxslt1-dev zlib1g-dev
  sudo apt install libjpeg-dev libpng-dev libtiff-dev
  sudo apt install libgl1-mesa-glx libgl1-mesa-dev
  sudo apt install libcairo2-dev
  ~~~

# 01. Vision server 
## A. GPU Driver Installation
### 1. Add the official graphics drivers repository:
  ~~~
  sudo add-apt-repository ppa:graphics-drivers/ppa
  sudo apt update
  reboot
  ~~~

### 2. Install GPU driver
  ~~~
  # stable version 
  sudo apt update
  sudo ubuntu-drivers devices
  sudo ubuntu-drivers autoinstall 
  reboot
  ~~~
  or
  ~~~
  # find the recommended version
  sudo ubuntu-drivers devices

  #output
  == /sys/devices/pci0000:00/.../0000:01:00.0 ==
  model    : NVIDIA GeForce RTX 4080
  vendor   : NVIDIA Corporation
  modalias : pci:v000010DEd000027B6sv00001028sd00002381bc03sc00i00
  driver   : nvidia-driver-535 - distro non-free **recommended**
  driver   : nvidia-driver-525 - distro non-free

  sudo apt install nvidia-driver-535
  sudo reboot
  ~~~

### 3. install CUDA Toolkit
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
~~~
  # 패키지 저장소 키 등록
  sudo apt update
  sudo apt install -y software-properties-common
  
  # NVIDIA 공식 repo 추가
  sudo add-apt-repository ppa:graphics-drivers/ppa
  
  # CUDA Toolkit 설치 (예: 12.2)
  sudo apt-get update
  sudo apt-get install -y nvidia-cuda-toolkit
  sudo reboot
~~~


## B. Docker 환경 설정  
### 1. install docker
~~~
# 필수 패키지 설치
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release

# Docker GPG 키 등록
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Docker 저장소 추가
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Docker Engine 설치
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# sudo 없이 실행하기
sudo usermod -aG docker $USER
reboot
~~~
### 2. install nvidia-docker
참고 링크: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

~~~angular2html
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo reboot
~~~

### 3. vision docker image 빌드
~~~angular2html
cd VISION/docker
docker build -t ketirobotvision/ketinet:v1 .

# docker image 빌드 확인 
docker images
~~~

### 4. docker 컨테이터 생성 및 내부 설정 
~~~
# host pc
./build_vision_container.sh

# 비전 컨테이너 실행 
run_vision
~~~

~~~
# 컨테이너 내부 설치
cd ~/volume/VISION/docker/    
./install.sh
~~~

# 03. Vision code
## Environment setting
### 가상환경 만들기 및 활성화
~~~
cd ${HOME} # keti-poc-talos 폴더로 이동
python3 -m venv venv
# system에서 python 기본 version이 3.8이 아닐 경우 
python3.8 -m venv venv
source venv/bin/activate

~~~
### 패키지 설치
~~~
cd ./VISION
sudo chmod -R 777 ./
cd ketisdk
pip install -e . 
cd ..

pip install -e pyconnect
pip install -e pyinterfaces
pip install torch==1.12.1+cu113 torchvision==0.13.1+cu113 torchaudio==0.12.1 --extra-index-url https://download.pytorch.org/whl/cu113
sudo apt install -y libgirepository1.0-dev libcairo2-dev 
pip install -r requirements.txt
~~~


## 실행
### vision docker 
~~~
run_vision
cd ~/volume/VISION
python3 run_server_talos.py
~~~

~~~
source ./venv/bin activate
python3 test_vision.py
~~~
~~~python
import sys,os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from interface.vision import *
import copy
import math
import numpy as np


if __name__ == '__main__':
    root_dir=__file__.split("test_vision")[0]
    # If connection fails:
    # 1. Run `ifconfig` inside the vision container
    # 2. Find the eth0 IP address
    # 3. Set HOST to that IP
    vision=Vision(HOST="172.17.0.2",PORT=8801)
    enable_onair=False
    if enable_onair:
        # realsense data 획득
        ret_rs,rgb,depth=vision.get_rs()
    else:
        imgpath = f"{root_dir}APP_Base/APP/data/dataset/2025_05_12_17_58_29__rgb.png"
        rgb=cv2.imread(imgpath)
        imgpath = f"{root_dir}APP_Base/APP/data/dataset/2025_05_12_17_58_29__depth.png"
        depth=cv2.imread(imgpath,-1)
    # grasping
    ret_pick,ret,ret_img=vision.get_pick(rgb,depth,
                            aDict={
                            # ROI (x, y, w, h) in image
                            "crop_roi": [450, 172, 1040, 513],

                            # Grasp width constraints (pixels)
                            "dmin": 50,   # minimum width
                            "dmax": 70,   # maximum width
                            "df": 50,

                            # Bounding box size ratio relative to image size
                            # ratio = object_width/img_width or object_height/img_height
                            # "min_ratio": 0.01,  # lower bound (currently disabled)
                            "max_ratio": 1,

                            # Object size constraints (pixel area)
                            "min_mass": 500,     # minimum object area
                            "max_mass": 10000,   # maximum object area

                            # Number of top candidate boxes to keep
                            "topn": 100,
                    }
                    )
    [Robot_X, Robot_Y, Robot_Z,robot_z_angle, ret_back]=ret
    print(f"[Robot_X, Robot_Y, Robot_Z]: [{Robot_X}, {Robot_Y}, {Robot_Z}]")
    print(f"robot_z_angle: {robot_z_angle}")
    cv2.imshow("ret_img",ret_img)
    cv2.waitKey(100)

    if enable_onair:
        # TIS RGB data 획득
        ret_tis,img,origin=vision.get_TIS()
    else:
        imgpath = f"{root_dir}APP_Base/APP/data/angle/2025_05_27_19_00_54__tis.png"
        origin=cv2.imread(imgpath)
    # angle data 획득
    ret_angle,ret,ret_img = vision.get_angle(origin,
                            aDict={
                            # ROI 설정 (x, y, w, h) in image
                            "roi":[330, 12, 1393, 1098],

                            # Blur kernel size for HSV preprocessing (must be odd).
                            # Larger values → smoother contours
                            "blur_size": 11,

                            # HSV threshold preprocessing
                            "max_h_threshold": 120,
                            "max_s_threshold": 100,
                            "max_v_threshold": 105,

                            # Circle detection parameters (cv2.HoughCircles 참고)
                            "param1": 30,
                            "param2": 40,
                            "minRadius": 140,
                            "maxRadius": 165,

                            # Polar coordinates angle range
                            "angle_range": [-120, 90],

                            # Median filter size for polar signal (must be odd)
                            "single_med_filtersize": 11,

                            # 기준 각도 (reference angle)
                            "ref_angle":-86.94130242590418
                            }
                            )

    print(f"ret['differ_angle']: {ret['differ_angle']}")
    cv2.imshow("ret_img",ret_img)
    cv2.waitKey(100)

~~~
