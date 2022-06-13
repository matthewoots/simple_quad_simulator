#include <string>
#include <mutex>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <random>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;



class quad_class
{
    private:
    ros::NodeHandle _nh;
    ros::Subscriber _pos_raw_sub, _log_traj_sub;
    ros::Publisher _odom_pub;
    ros::Publisher _mesh_pub, _cmd_pub, _log_path_pub, _log_traj_pub;

    ros::Timer _drone_timer;

    mavros_msgs::PositionTarget _cmd;

    double _init_x, _init_y, _init_z;

    bool _received_cmd = false;
    bool lerp_update;
    int uav_id;
    int interval_div, interval_count;
    std::string _id, _mesh_resource;
    double _simulation_interval, _command_interval, _state_pub_rate, _command_rate;
    double _timeout, _yaw_offset, last_yaw;
    Eigen::Vector3d _start;

    Eigen::Vector3d f_c_pos, f_c_vel, f_c_acc;
    Eigen::Vector3d p_c_pos, p_c_vel, p_c_acc;

    std::mutex command_mutex;
    std::mutex odometry_mutex;
    std::mutex trajectory_mutex;

    visualization_msgs::Marker meshROS;

    ros::Time log_previous_time;
    nav_msgs::Path path;

    /** @brief using http://wiki.ros.org/mavros/CustomModes
    * @param AUTO.MISSION
    * @param AUTO.LOITER
    */
    std::string mavros_custom_mode;

    Eigen::Vector4d color_vect;

    public:

    struct state_command
    {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d acc;
        Eigen::Quaterniond q;
        double t;
    };

    ros::Time command_time;

    quad_class::state_command sc;

    vector<quad_class::state_command> traj_vector;
    vector<Eigen::Vector3d> traj_pos_vector;

    nav_msgs::Odometry odom;

	quad_class(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
	{
        _nh.param<std::string>("mesh_resource", _mesh_resource, "");
        _nh.param<std::string>("agent_id", _id, "drone0");
		_nh.param<double>("state_pub_rate", _state_pub_rate, 1.0);
        _nh.param<double>("command_rate", _command_rate, 1.0);

        _nh.param<double>("timeout", _timeout, 0.5);

        _nh.param<double>("start_x", _start(0), 0.0);
        _nh.param<double>("start_y", _start(1), 0.0);
        _nh.param<double>("start_z", _start(2), 0.0);
        _nh.param<double>("yaw_offset", _yaw_offset, 0.0);

		_simulation_interval = 1 / _state_pub_rate;
        _command_interval = 1 / _command_rate;

		std::string copy_id = _id; 
		string uav_id_char = copy_id.erase(0,5); // removes first 5 character
		uav_id = stoi(uav_id_char);

        /** @brief Subscriber that receives control raw setpoints via mavros */
		_pos_raw_sub = _nh.subscribe<mavros_msgs::PositionTarget>(
			"/" + _id + "/mavros/setpoint_raw/local", 20, &quad_class::command_callback, this);
        _log_traj_sub = _nh.subscribe<trajectory_msgs::JointTrajectory>(
            "/trajectory/points", 200, &quad_class::trajectory_callback, this);

		/** @brief Publisher that publishes odometry data */
		_odom_pub = _nh.advertise<nav_msgs::Odometry>(
			"/" + _id + "/mavros/odom_nwu", 10);

        
        /** @brief For visualization */
        /** @brief Publisher that publishes rviz_visualization data */    
        _mesh_pub = _nh.advertise<visualization_msgs::Marker>(
            "/" + _id + "/uav/mesh", 10, true);
        _cmd_pub = _nh.advertise<geometry_msgs::PointStamped>(
            "/" + _id + "/uav/target", 10, true);
        _log_path_pub = _nh.advertise<nav_msgs::Path>(
            "/" + _id + "/uav/log_path", 10, true);
        _log_traj_pub = _nh.advertise<visualization_msgs::Marker>(
            "/" + _id + "/uav/log_trajectory", 10, true);


		/** @brief Timer that handles drone state at each time frame */
		_drone_timer = _nh.createTimer(ros::Duration(_simulation_interval), &quad_class::drone_timer, this, false, false);

        printf("%sdrone%d%s start_pose [%s%.2lf %.2lf %.2lf%s]! \n", 
            KGRN, uav_id, KNRM,
            KBLU, _start(0), _start(1), _start(2), KNRM);

        odom.pose.pose.position.x = _start(0);
	    odom.pose.pose.position.y = _start(1);
	    odom.pose.pose.position.z = _start(2);

        p_c_pos = f_c_pos = _start;
        p_c_vel = f_c_vel = Eigen::Vector3d::Zero();
        p_c_acc = f_c_acc = Eigen::Vector3d::Zero();

        interval_div = (int)floor(_simulation_interval / _command_interval);
        interval_count = interval_div;

	    odom.pose.pose.orientation.w = 1.0;
	    odom.pose.pose.orientation.x = 0.0;
	    odom.pose.pose.orientation.y = 0.0;
	    odom.pose.pose.orientation.z = 0.0;

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;

        sc.pos = Eigen::Vector3d(_start(0), _start(1), _start(2));
        sc.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
        sc.acc = Eigen::Vector3d(0.0, 0.0, 0.0);
        sc.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

        // Choose a color for the trajectory using random values
        std::random_device dev;
        std:mt19937 generator(dev());
        std::uniform_real_distribution<double> dis(0.0, 1.0);
        color_vect = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);

		printf("%sdrone%d%s created! \n", KGRN, uav_id, KNRM);
        _drone_timer.start();
    }

	~quad_class()
    {
        _drone_timer.stop();
    }

    bool check_sent_command(double tolerance);

    void drone_timer(const ros::TimerEvent &);

    void update_odom();

    void stop_and_hover();

    /** @brief callbacks */
    void command_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

    /** @brief visualization functions */
    void visualize_uav();
    void visualize_log_path();
    void visualize_traj_path();

    /** @brief common utility functions */
    vector<quad_class::state_command> joint_trajectory_to_state(
        trajectory_msgs::JointTrajectory jt);
    Eigen::Quaterniond calc_uav_orientation(
        Eigen::Vector3d acc, double yaw_rad);
    void display_marker_list(
        vector<Eigen::Vector3d> vect, double scale,
        Eigen::Vector4d color, int id);
};