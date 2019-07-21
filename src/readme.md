# Method1
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git checkout kinetic-devel
cd $DynamixelSDK-kinetic-devel/c++/build/linux64
make && make install


# Method2
```
sudo apt-get install ros-kinetic-dynamixel-sdk
sudo apt-get install ros-kinetic-dynamixel-workbench
```

## use manual
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/
start controller
start operator


[ID] 2, [Model Name] MX-28, [Protocol Version] 1.0, [BAUD RATE] 57600

## controller
controller yaml template:
rosed dynamixel_workbench_controllers dynamixel_controllers.launch
vim /opt/ros/kinetic/share/dynamixel_workbench_controllers/config/basic.yaml

control modes:
http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
3 is position control

## operator
root@kaanh-laptop:/home/kaanh# rosed dynamixel_workbench_operators joint_operator.launch
/opt/ros/kinetic/share/dynamixel_workbench_operators/config/motion.yaml

motion.yaml