USB connect roslaunch mavros apm.launch fcu_url:=/dev/ttyUSB0:115200

unlock arm 

rosservice call /mavros/cmd/arming "value: true"
rosservice call /mavros/cmd/arming "value: false"

rosservice call /mavros/set_mode "custom_mode: 'ALT_HOLD'"

check 

rostopic echo /mavros/state

armed: True
mode: ALT_HOLD

lauch
roslaunch /home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/launch/start_dobmpc_tank.launch

run bridge

rosrun /home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/sim2_real/sim2real.py



working
rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'MANUAL'" 
rosrun mavros mavsys mode -c MANUAL

rostopic echo /mavros/state

rostopic pub -r 20 mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "{header: auto, twist: {linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0.0, y: 0.0, z: 0}}}"

rostopic pub -r 20 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 

roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555
  
