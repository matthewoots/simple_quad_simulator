import sys
import numpy as np
import os
import math

# Format is
# python gen_launch_w_trajectory.py 10 antipodal <radius>
# python gen_launch_w_trajectory.py 10 horizontal-line  <space from center> <spacing>
# python gen_launch_w_trajectory.py 10 vertical-line  <space from center> <spacing>
# python gen_launch_w_trajectory.py 10 top-down-facing <space from center> <spacing>
# python gen_launch_w_trajectory.py 10 left-right-facing <space from center> <spacing>

# Acceptable formations are 
# 1. antipodal
# 2. horizontal-line
# 3. vertical-line
# 4. top-down-facing
# 5. left-right-facing

def main(argv):
	num = int(argv[1])
        _formation = str(argv[2])
        height = 1.5

        if _formation == "antipodal":
          
          theta = np.linspace(-3.14159, 3.14159, num+1)
          r = int(argv[3])
          
        if _formation == "vertical-line":
          first_row = int(math.floor(num))
          first_row_offset = (first_row-1) * float(argv[4]) / 2

        if _formation == "horizontal-line":
          first_row = int(math.floor(num))
          first_row_offset = (first_row-1) * float(argv[4]) / 2

        if _formation == "top-down-facing":
          first_row = int(math.floor(num / 2))
          second_row = num - first_row
          first_row_offset = (first_row-1) * float(argv[4]) / 2
          second_row_offset = (second_row-1) * float(argv[4]) / 2

        if _formation == "left-right-facing":
          first_row = int(math.floor(num / 2))
          second_row = num - first_row
          first_row_offset = (first_row-1) * float(argv[4]) / 2
          second_row_offset = (second_row-1) * float(argv[4]) / 2

        fname = "../launch/generated_" + _formation + \
          "_" + str(num) + ".launch"

	str_begin = "<?xml version=\"1.0\"?>\n\
<launch>\n"

	str_end = "\n\
  </launch>\n"

	print("number of agents : {x}".format(x=num))
	file = open(fname, "w")
	file.write(str_begin)
	for i in range(0,num):
            if _formation == "antipodal":
              _start_x = r * math.sin(theta[i]) 
              _start_y = r * math.cos(theta[i])
              
            if _formation == "horizontal-line":
              _start_x = float(argv[4])*(i) - first_row_offset
              _start_y = float(argv[3])

            if _formation == "vertical-line":
              _start_x = float(argv[3])
              _start_y = float(argv[4])*(i) - first_row_offset
            
            if _formation == "top-down-facing":
              # second row down
              if (i + 1) > first_row:
                _start_x = - float(argv[3])
                _start_y = float(argv[4])*(i-first_row) - second_row_offset
              else:
                _start_x = float(argv[3])
                _start_y = float(argv[4])*(i) - first_row_offset

            if _formation == "left-right-facing":
              # second row right
              if (i + 1) > first_row:
                _start_x = float(argv[4])*(i-first_row) - second_row_offset
                _start_y = - float(argv[3])
              else:
                _start_x = float(argv[4])*(i) - first_row_offset
                _start_y = float(argv[3])

            agent = i
            print("written times {x}".format(x=agent))
            str_for_agent = "\n\
<group ns=\"drone{drone_ns}\">\n\
    <arg name=\"id\" value=\"drone{drone_id}\"/>\n\
    <arg name=\"command_hz\" value=\"10\"/>\n\
    <node pkg=\"quad\" type=\"quad_node\" name=\"quad_node\" output=\"screen\">\n\
        <rosparam command=\"load\" file=\"$(find quad)/params/param.yaml\" />\n\
        <param name=\"mesh_resource\" value=\"file://$(find quad)/meshes/fake_drone.dae\" />\n\
        <param name=\"agent_id\" value=\"$(arg id)\"/>\n\
        <param name=\"sensing_range\" value=\"4\"/>\n\
        <param name=\"command_rate\" value=\"$(arg command_hz)\"/>\n\
        <param name=\"start_x\" value=\"{start_x}\"/>\n\
        <param name=\"start_y\" value=\"{start_y}\"/>\n\
        <param name=\"start_z\" value=\"{start_z}\"/>\n\
        <param name=\"yaw_offset\" value=\"{yaw_offset}\"/>\n\
    </node>\n\
    <remap from=\"/$(arg id)/mavros/local_position/pose\" to=\"/$(arg id)/mavros/odom_nwu\"/>\n\
    <node pkg=\"bs_trajectory_ros\" type=\"bs_trajectory_ros_node\" name=\"bs_trajectory_ros_node\" output=\"screen\">\n\
        <param name=\"agent_id\" value=\"$(arg id)\"/>\n\
        <param name=\"odom_or_pose\" value=\"odom\"/>\n\
        <param name=\"traj_opt_update_hz\" value=\"3\"/>\n\
        <param name=\"cmd_update_hz\" value=\"$(arg command_hz)\"/>\n\
        <!-- Bspline parameters -->\n\
        <param name=\"traj_duration_secs\" value=\"8.0\"/>\n\
        <param name=\"order\" value=\"4\"/>\n\
        <param name=\"des_knot_div\" value=\"4\"/>\n\
        <param name=\"max_velocity\" value=\"1.5\"/>\n\
        <!-- RRT parameters -->\n\
        <param name=\"runtime_error\" value=\"0.12\"/>\n\
        <param name=\"sub_runtime_error\" value=\"0.025\"/>\n\
        <param name=\"search_radius\" value=\"7.0\"/>\n\
        <param name=\"threshold\" value=\"0.40\"/>\n\
        <param name=\"search_interval\" value=\"7.0\"/>\n\
        <param name=\"xybuffer\" value=\"1.5\"/>\n\
        <param name=\"zbuffer\" value=\"1.5\"/>\n\
        <param name=\"passage_size\" value=\"6.0\"/>\n\
        <rosparam param=\"height\">[1.0, 3.0]</rosparam>\n\
        <!-- Optimization parameters -->\n\
        <param name=\"weight_smooth\" value=\"0.0008\"/>\n\
        <param name=\"weight_feas\" value=\"0.0075\"/>\n\
        <param name=\"weight_static\" value=\"1.50\"/>\n\
        <param name=\"weight_reci\" value=\"0.0200\"/>\n\
        <param name=\"max_acc\" value=\"0.4\"/>\n\
    </node>\n\
</group>\n\
    \n".format(drone_ns=agent, \
          drone_id=agent, \
          start_x=_start_x, \
          start_y=_start_y, \
          start_z=height, \
          yaw_offset=0.0)
            file.write(str_for_agent)
        
	file.write(str_end)
	file.close()

if __name__ == '__main__':
	main(sys.argv)