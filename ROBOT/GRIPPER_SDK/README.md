***
## Environment

### Linux Version : Ubuntu 22.04
***

## 1. Update OS
~~~
sudo apt-get update && sudo apt-get upgrade
~~~

## 2. Install dependency libraries
~~~
sudo apt-get install libmodbus*

sudo apt install python3-pip python3-dev
pip3 install pymodbus
~~~

## 3. Run example
### 3.1 C++
~~~
g++ -std=c++11 -O2 example/example.cpp \
  -I./include -L./lib -lzimmergripper \
  -Wl,-rpath,'$ORIGIN/lib' \
  -o demo
./demo
~~~

### 3.2 Python
~~~
python3 example/zimmergripper.py 'zimmergripper.h include path' 'libzimmergripper.so file path'
ex : python3 example/zimmergripper.py include lib/libzimmerbripper.so
~~~

## 4. Functions description
### 4.1 Connect
~~~
Connect to the I/O link master connected to the zimmer gripper

Parameters
----------
ip : string
    IP address of I/O link master.
port : int
    Port number of I/O link master. Fixed to 502.

Returns
----------
This function has no return value.

Example
----------
gripper.Connect('192.168.137.254', 502)
~~~
### 4.2 Disconnect
~~~
Disconnect to the I/O link master.

Parameters
----------
This function has no input parameter and return value.
~~~
### 4.3 Init
~~~
Initialize I/O link master to operate to gripper.

Parameters
----------
This function has no input parameter and return value.
~~~
### 4.4 Grip
~~~
Move gripper toolip in the direction of grasping

Parameters
----------
This function has no input parameter and return value.
~~~
### 4.5 Release
~~~
Move gripper toolip in the direction of releasing

Parameters
----------
This function has no input parameter and return value.
~~~
### 4.6 isConnected
~~~
Check the connect status with I/O link master.

Parameters
----------
This function has no input parameter.

Returns
----------
boolean
    Connected status with I/O link master.
~~~
### 4.7 Move
~~~
Move gripper tooptip to the desired position.

Parameters
----------
position : int
    The desired position value.

Returns
----------
This function has no return value.
~~~