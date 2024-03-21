Autonomous Navigation & Real Time Mapping on iOS
```
--------------------------------------------------------------------------------
To run Lane following:
change the name of the bot in default.sh in launcher
dts devel build -f
dts devel run -R [Vehicle_Name]
--------------------------------------------------------------------------------
To run Analysis & Trajectory:
1 Keyboard Control:		
	dts duckiebot keyboard_ control
2 Camera Control:			
	dts start_gut_tools 'duckiebotname' 
	rqt_image_view duckiebotname'
3 Run Trajectory Calculation: 
	dts devel build -f 
	dts devel run -R duckiebotname' -L trajectory
4 See published Messages: 
	dts devel run -H duckiebotname' -L my-subscriber
--------------------------------------------------------------------------------
# Mobile ROS

Mobile ROS is a iOS application for communication of iOS devices and ROS robots.

## Installation

Use the package manager [npm](https://www.npmjs.com) to install dependencies of Mobile ROS, make sure you are under root path of the project.

bash
npm install


## Usage - Server side
You need to install [ROS bridge](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge#Installing_Rosbridge) on the robot to establish connections by executing

bash
sudo apt-get install ros-<rosdistro>-rosbridge-suite

After the installation, source the setup bash file by following

bash
source /opt/ros/<rosdistro>/setup.bash

you need to launch rosbridge every time to use

bash
roslaunch rosbridge_server rosbridge_websocket.launch

## Usage - Client side
to build and start the app
bash
npx expo start

 then press i/a/w to launch iOS/Android/web version of the application, you may install expo go if you want to use it on iOS

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
