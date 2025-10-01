#!/bin/bash
echo "=========================="
echo "1. install RGB camera sdk "
echo "=========================="
apt-get update
apt-get install -y git cmake build-essential libgstreamer1.0-dev \
                   libgstreamer-plugins-base1.0-dev libglib2.0-dev \
                   libudev-dev

apt-get install -y udev usbutils v4l-utils \
                   gstreamer1.0-tools gstreamer1.0-plugins-base \
                   gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                   python3 python3-pip
                   

cd ~/volume/VISION/tiscamera/               
./scripts/dependency-manager install
mkdir build
cd build
cmake ..
make -j
cd ..
source ./build/env.sh
echo "source ~/volume/VISION/tiscamera/build/env.sh" >> ~/.bashrc

echo "=========================="
echo "2. Run RGB camera viewer  "
echo "=========================="
tcam-ctrl -l
tcam-capture

echo "=========================="
echo "3. install realsense sdk  "
echo "=========================="
apt-get update

mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
apt-get update
apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg
pip3 install --upgrade pip
apt-get install -y python3-pyrealsense2 || true

echo "=========================="
echo "4. Run realsense viewer   "
echo "=========================="
rs-enumerate-devices
realsense-viewer

echo "=========================="
echo "5. Vision sdk   "
echo "=========================="

mkdir /root/pkgs
cd /root/pkgs
git clone https://github.com/CASIA-IVA-Lab/FastSAM.git
cd FastSAM
pip install -e .

cd ~/volume/VISION
pip install wheel
pip install -e ./pyconnect
pip install -e ./pyinterfaces
pip install -e ./pyrecognition
cd ketisdk
pip install -e . 
cd .. 
pip install -r requirements.txt
