# Simple Quad Simulator ROS
ROS-Rviz Quadcopter playground to test path planning and mapping algorithms (ROS package is called `quad`)

### Setup
```bash
# Git clone this repo into your ws/src directory
cd <Workspace-Source-Directory>
git clone https://github.com/matthewoots/simple_quad_simulator.git
# Go to your ws directory
cd <Workspace-Directory>
catkin build
source devel/setup.bash
```

### Quick run
```bash
roslaunch quad sample.launch
```

### Setup multiple drone profiles
1. Generate display in rviz
```bash
cd simple_quad_simulator/scripts
python gen_rviz_display.py <number-of-drones>
# The file will be generated in simple_quad_simulator/rviz directory
# etc display_40.rviz
```

### Test functionality of drone messages
```bash
cd simple_quad_simulator/scripts
# Test whether single agent can receive /mavros/setpoint_raw/local (Target message)
python test_target.py 
# Test whether single agent can receive /trajectory/points
python test_trajectory.py 
```

## References 
1. Swarm-playground https://github.com/ZJU-FAST-Lab/EGO-Planner-v2