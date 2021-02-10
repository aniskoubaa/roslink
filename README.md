# ROSLink

ROSLink is a new protocol to integrate Robot Operating System (ROS) enabled-robots with the IoT. The motivation behind ROSLink is the lack of ROS functionality in monitoring and controlling robots through the Internet.

For more details please see: [ROSLink paper](https://link.springer.com/chapter/10.1007/978-3-319-54927-9_8)

## Compatibility
This version is deployed and tested on ROS Kientic and ROS Melodic which both use Python 2.7.
Note that this code is not compatible for ROS Noetic and its Python 3.x.
For the roslink version working under ROS Noetic and Python 3, consider the correct branch ros-noetic-python3

## License

* This software is provided without warranty.
* The software author or license can not be held liable for any damages inflicted by the software.
* This software is free to use for academic research and education noncommercial purposes with not-for-profit usage.
* Any modification of the software for research, education and not-for-profit usage should be also pubically shared
* For commercial for-profit usage, a commercial license has to be requested from the author of the code Anis Koubaa. The commercial license will allow to use privately the modified code without having to open source the modifications. The commercial license does not prevent the author from releasing newer versions of ROSLink without prior notice. 


## Installation
To install and compile roslink open terminal and type
```shell
cd path/to/catkin_workspace/src 
git clone https://github.com/aniskoubaa/roslink.git
cd ..
catkin_make
```
Install simple websocket server pacakge which is needed for ROSLink bridge and `proxy-server.py` that acts as cloud to manage the communication between the robot and the user  
`pip install git+https://github.com/dpallot/simple-websocket-server.git`

## Dependencies
install the following dependencies (use pip3 is you have Python 3.x)
```shell
pip install websocket
pip install websocket-client
```
## Usage
- Run the proxy server using `python proxy-server.py`
- In `src/tb3/tb3-roslink.launch` file: 
  - change `map_location` parm to the map image path
  - change `ground_station_ip` and `ground_station_port` parms to be same as the proxy server
- The last thing is to launch the rolsink bridge  `roslaunch roslink tb3-roslink.launch`

## Running TB3 over Internet
only some changes needed:
- Copy `proxy-server.py` file to you public cloud.
- install simple websocket server pacakge on the cloud
- Run the proxy server on the cloud using `python proxy-server.py`
- In `src/tb3/tb3-roslink.launch` file, change `ground_station_ip` parm to the cloud IP address
- Run roslink bridge as previous.


## Tutorials
[[RIOTU] The Internet of-Unmanned Systems using ROS](https://www.youtube.com/watch?v=Om8tCDZieGI), for more information about the package watch this video 

## Udemy Courses 
[Check Prof. Anis Koubaa Udemy courses to learn ROS](https://www.riotu-lab.org/udemy.php)
  
