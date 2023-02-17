import sys
import numpy as np
import os
import math

def main(argv):
	num = int(argv[1])
        fname = "../rviz/generated_display_" + str(num) + ".rviz"

	str_begin = "\n\
Panels:\n\
  - Class: rviz/Displays\n\
    Help Height: 78\n\
    Name: Displays\n\
    Property Tree Widget:\n\
      Expanded:\n\
        - /Global Options1\n\
      Splitter Ratio: 0.45588234066963196\n\
    Tree Height: 707\n\
  - Class: rviz/Selection\n\
    Name: Selection\n\
  - Class: rviz/Tool Properties\n\
    Expanded:\n\
      - /2D Pose Estimate1\n\
      - /2D Nav Goal1\n\
      - /Publish Point1\n\
    Name: Tool Properties\n\
    Splitter Ratio: 0.5886790156364441\n\
  - Class: rviz/Views\n\
    Expanded:\n\
      - /Current View1\n\
    Name: Views\n\
    Splitter Ratio: 0.5\n\
  - Class: rviz/Time\n\
    Name: Time\n\
    SyncMode: 0\n\
    SyncSource: \"global_map\"\n\
Preferences:\n\
  PromptSaveOnExit: true\n\
Toolbars:\n\
  toolButtonStyle: 2\n\
Visualization Manager:\n\
  Class: \"\"\n\
  Displays:\n\
    - Alpha: 0.5\n\
      Cell Size: 1\n\
      Class: rviz/Grid\n\
      Color: 160; 160; 164\n\
      Enabled: true\n\
      Line Style:\n\
        Line Width: 0.029999999329447746\n\
        Value: Lines\n\
      Name: Grid-1m\n\
      Normal Cell Count: 0\n\
      Offset:\n\
        X: 0\n\
        Y: 0\n\
        Z: 0\n\
      Plane: XY\n\
      Plane Cell Count: 100\n\
      Reference Frame: <Fixed Frame>\n\
      Value: true\n\
    - Alpha: 0.5\n\
      Cell Size: 10\n\
      Class: rviz/Grid\n\
      Color: 204; 0; 0\n\
      Enabled: true\n\
      Line Style:\n\
        Line Width: 0.029999999329447746\n\
        Value: Lines\n\
      Name: Grid-10m\n\
      Normal Cell Count: 0\n\
      Offset:\n\
        X: 0\n\
        Y: 0\n\
        Z: 0\n\
      Plane: XY\n\
      Plane Cell Count: 10\n\
      Reference Frame: <Fixed Frame>\n\
      Value: true\n\
        \n\
            "

	str_end = "\n\
    - Class: rviz/Group\n\
      Displays:\n\
        - Alpha: 1\n\
          Autocompute Intensity Bounds: true\n\
          Autocompute Value Bounds:\n\
            Max Value: 10\n\
            Min Value: -10\n\
            Value: true\n\
          Axis: Z\n\
          Channel Name: intensity\n\
          Class: rviz/PointCloud2\n\
          Color: 32; 74; 135\n\
          Color Transformer: FlatColor\n\
          Decay Time: 0\n\
          Enabled: true\n\
          Invert Rainbow: false\n\
          Max Color: 255; 255; 255\n\
          Min Color: 0; 0; 0\n\
          Name: global_map\n\
          Position Transformer: XYZ\n\
          Queue Size: 10\n\
          Selectable: true\n\
          Size (Pixels): 3\n\
          Size (m): 0.009999999776482582\n\
          Style: Points\n\
          Topic: /global_map\n\
          Unreliable: false\n\
          Use Fixed Frame: true\n\
          Use rainbow: true\n\
          Value: true\n\
      Enabled: true\n\
      Name: environment\n\
  Enabled: true\n\
  Global Options:\n\
    Background Color: 238; 238; 236\n\
    Default Light: true\n\
    Fixed Frame: world\n\
    Frame Rate: 30\n\
  Name: root\n\
  Tools:\n\
    - Class: rviz/Interact\n\
      Hide Inactive Objects: true\n\
    - Class: rviz/MoveCamera\n\
    - Class: rviz/Select\n\
    - Class: rviz/FocusCamera\n\
    - Class: rviz/Measure\n\
    - Class: rviz/SetInitialPose\n\
      Theta std deviation: 0.2617993950843811\n\
      Topic: /initialpose\n\
      X std deviation: 0.5\n\
      Y std deviation: 0.5\n\
    - Class: rviz/SetGoal\n\
      Topic: /move_base_simple/goal\n\
    - Class: rviz/PublishPoint\n\
      Single click: true\n\
      Topic: /clicked_point\n\
  Value: true\n\
  Views:\n\
    Current:\n\
      Class: rviz/Orbit\n\
      Distance: 52.870243072509766\n\
      Enable Stereo Rendering:\n\
        Stereo Eye Separation: 0.05999999865889549\n\
        Stereo Focal Distance: 1\n\
        Swap Stereo Eyes: false\n\
        Value: false\n\
      Field of View: 0.7853981852531433\n\
      Focal Point:\n\
        X: 0\n\
        Y: 0\n\
        Z: 0\n\
      Focal Shape Fixed Size: false\n\
      Focal Shape Size: 0.05000000074505806\n\
      Invert Z Axis: false\n\
      Name: Current View\n\
      Near Clip Distance: 0.009999999776482582\n\
      Pitch: 1.5697963237762451\n\
      Target Frame: <Fixed Frame>\n\
      Value: Orbit (rviz)\n\
      Yaw: 3.1414999961853027\n\
    Saved: ~\n\
Window Geometry:\n\
  Displays:\n\
    collapsed: false\n\
  Height: 1013\n\
  Hide Left Dock: false\n\
  Hide Right Dock: true\n\
  QMainWindow State: 000000ff00000000fd00000004000000000000015600000361fc0200000032fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d00000361000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000fb0000000a006400650070007400680000000000ffffffff0000000000000000000000010000010f00000363fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d00000363000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007490000003efc0100000002fb0000000800540069006d0065010000000000000749000002eb00fffffffb0000000800540069006d00650100000000000004500000000000000000000005ed0000036100000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000\n\
  Selection:\n\
    collapsed: false\n\
  Time:\n\
    collapsed: false\n\
  Tool Properties:\n\
    collapsed: false\n\
  Views:\n\
    collapsed: true\n\
  Width: 1868\n\
  X: 52\n\
  Y: 30\n"

	print("number of agents : {x}".format(x=num))
	file = open(fname, "w")
	file.write(str_begin)
	for i in range(0,num):
            agent = i
            print("written times {x}".format(x=agent))
            str_for_agent = "\n\
    - Class: rviz/Group\n\
      Displays:\n\
        - Class: rviz/Marker\n\
          Enabled: true\n\
          Marker Topic: /drone{drone_id_mesh}/uav/mesh\n\
          Name: uav_mesh\n\
          Namespaces:\n\
            drone: true\n\
          Queue Size: 1\n\
          Value: true\n\
        - Angle Tolerance: 0.10000000149011612\n\
          Class: rviz/Odometry\n\
          Covariance:\n\
            Orientation:\n\
              Alpha: 0.5\n\
              Color: 255; 255; 127\n\
              Color Style: Unique\n\
              Frame: Local\n\
              Offset: 1\n\
              Scale: 1\n\
              Value: true\n\
            Position:\n\
              Alpha: 0.30000001192092896\n\
              Color: 204; 51; 204\n\
              Scale: 1\n\
              Value: true\n\
            Value: true\n\
          Enabled: true\n\
          Keep: 1\n\
          Name: uav_odom\n\
          Position Tolerance: 0.10000000149011612\n\
          Queue Size: 10\n\
          Shape:\n\
            Alpha: 1\n\
            Axes Length: 0.30000001192092896\n\
            Axes Radius: 0.05000000074505806\n\
            Color: 255; 25; 0\n\
            Head Length: 0.30000001192092896\n\
            Head Radius: 0.10000000149011612\n\
            Shaft Length: 1\n\
            Shaft Radius: 0.05000000074505806\n\
            Value: Axes\n\
          Topic: /drone{drone_id_odom}/mavros/odom_nwu\n\
          Unreliable: false\n\
          Value: true\n\
        - Alpha: 0.5\n\
          Class: rviz/PointStamped\n\
          Color: 204; 41; 204\n\
          Enabled: true\n\
          History Length: 1\n\
          Name: uav_target\n\
          Radius: 0.10000000149011612\n\
          Topic: /drone{drone_id_target}/uav/target\n\
          Unreliable: false\n\
          Value: true\n\
        - Alpha: 1\n\
          Buffer Length: 1\n\
          Class: rviz/Path\n\
          Color: 52; 101; 164\n\
          Enabled: true\n\
          Head Diameter: 0.30000001192092896\n\
          Head Length: 0.20000000298023224\n\
          Length: 0.30000001192092896\n\
          Line Style: Lines\n\
          Line Width: 0.029999999329447746\n\
          Name: log_path\n\
          Offset:\n\
            X: 0\n\
            Y: 0\n\
            Z: 0\n\
          Pose Color: 255; 85; 255\n\
          Pose Style: None\n\
          Queue Size: 10\n\
          Radius: 0.029999999329447746\n\
          Shaft Diameter: 0.10000000149011612\n\
          Shaft Length: 0.10000000149011612\n\
          Topic: /drone{drone_id_log_path}/uav/log_path\n\
          Unreliable: false\n\
          Value: true\n\
        - Class: rviz/Marker\n\
          Enabled: true\n\
          Marker Topic: /drone{drone_id_trajectory}/uav/log_trajectory\n\
          Name: log_trajectory\n\
          Namespaces:\n\
            {{}}\n\
          Queue Size: 100\n\
          Value: true\n\
        - Alpha: 1\n\
          Autocompute Intensity Bounds: true\n\
          Autocompute Value Bounds:\n\
            Max Value: 10\n\
            Min Value: -10\n\
            Value: true\n\
          Axis: Z\n\
          Channel Name: intensity\n\
          Class: rviz/PointCloud2\n\
          Color: 204; 0; 0\n\
          Color Transformer: FlatColor\n\
          Decay Time: 0\n\
          Enabled: true\n\
          Invert Rainbow: false\n\
          Max Color: 255; 255; 255\n\
          Min Color: 0; 0; 0\n\
          Name: local_map\n\
          Position Transformer: XYZ\n\
          Queue Size: 10\n\
          Selectable: true\n\
          Size (Pixels): 4\n\
          Size (m): 0.009999999776482582\n\
          Style: Points\n\
          Topic: /drone{drone_id_pcl}/map\n\
          Unreliable: false\n\
          Use Fixed Frame: true\n\
          Use rainbow: true\n\
          Value: true\n\
      Enabled: true\n\
      Name: drone{drone_id}\n\
    ".format(drone_id_mesh=agent, \
          drone_id_odom=agent, \
          drone_id_target=agent, \
          drone_id_log_path=agent, \
          drone_id_trajectory=agent, \
          drone_id_pcl=agent, \
          drone_id=agent)
            file.write(str_for_agent)
        
	file.write(str_end)
	file.close()

if __name__ == '__main__':
	main(sys.argv)