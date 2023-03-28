#include <string>
#include <mutex>
#include <math.h>
#include <random>
#include <cmath>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>

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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/JointState.h>

#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>

#include <zju_laser_sim.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class quad_class
{
    private:

        struct state
        {
            double t;
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            Eigen::Quaterniond q;
            double yaw;
        };

        ros::NodeHandle _nh;

        ros::Subscriber _pos_raw_sub; 
        ros::Subscriber _log_traj_sub; 
        ros::Subscriber _full_pcl_sub;

        ros::Publisher _odom_pub, _local_pcl_pub, _agent_pub, _pose_pub;
        ros::Publisher _mesh_pub, _cmd_pub, _log_path_pub, _log_traj_pub;

        ros::Timer _drone_timer;
        ros::Timer _map_timer;

        mavros_msgs::PositionTarget _cmd;

        
        
        int uav_id;
        
        std::string _id; 
        std::string _mesh_resource;
        std::string uav_id_char;
        std::string _map_path;

        double _init_x;
        double _init_y;
        double _init_z;
        double _simulation_interval;
        double _simulation_rate; 
        double _map_interval;
        double _map_pub_rate;
        double _pose_pub_rate;
        double _timeout;
        double _yaw_offset;
        Eigen::Vector3d _start;

        quad_class::state _previous, _desired, _current;
        Eigen::Vector3d f_c_pos, f_c_vel, f_c_acc;
        Eigen::Vector3d p_c_pos, p_c_vel, p_c_acc;

        pcl::PointCloud<pcl::PointXYZ> _global_map;  

        std::mutex command_mutex;
        std::mutex odometry_mutex;
        std::mutex trajectory_mutex;

        ros::Time log_previous_time;
        ros::Time last_pose_time;
        nav_msgs::Path path;

        /** @brief using http://wiki.ros.org/mavros/CustomModes
        * @param AUTO.MISSION
        * @param AUTO.LOITER
        */
        std::string mavros_custom_mode;

        Eigen::Vector4d color_vect;

        laser::laser_simulator _laser;

    public:

        

        ros::Time command_time;

        std::vector<quad_class::state> traj_vector;

        std::vector<Eigen::Vector3d> traj_pos_vector;

        nav_msgs::Odometry odom;

        quad_class(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            int _hrz_laser_line_num;
            int _vtc_laser_line_num;
            double _resolution;
            double _sensing_range;
            double _hrz_laser_range_dgr;
            double _vtc_laser_range_dgr;

            _nh.param<std::string>("agent/mesh_resource", _mesh_resource, "");
            _nh.param<std::string>("agent/id", _id, "");
            _nh.param<double>("agent/simulation_rate", _simulation_rate, -1.0);
            _nh.param<double>("agent/pose_pub_rate", _pose_pub_rate, -1.0);
            
            _nh.param<double>("agent/sensing_range", _sensing_range, -1.0);
            _nh.param<double>("agent/timeout", _timeout, -1.0);
            _nh.param<double>("agent/start_x", _start(0), 0.0);
            _nh.param<double>("agent/start_y", _start(1), 0.0);
            _nh.param<double>("agent/start_z", _start(2), 0.0);
            _nh.param<double>("agent/yaw_offset", _yaw_offset, 0.0);

            _nh.param<std::string>("map/path", _map_path, "");
            _nh.param<double>("map/map_rate", _map_pub_rate, 0.0);
            _nh.param<double>("map/resolution", _resolution, 0.0);
            _nh.param<int>("map/horizontal/laser_line_num", _hrz_laser_line_num, 0);
            _nh.param<int>("map/vertical/laser_line_num", _vtc_laser_line_num, 0);
            _nh.param<double>("map/horizontal/laser_range_dgr", _hrz_laser_range_dgr, 0.0);
            _nh.param<double>("map/vertical/laser_range_dgr", _vtc_laser_range_dgr, 0.0);

            _simulation_interval = 1 / _simulation_rate;
            _map_interval = 1 / _map_pub_rate;

            std::string copy_id = _id; 
            uav_id_char = copy_id.erase(0,5); // removes first 5 character
            uav_id = std::stoi(uav_id_char);

            /** @brief Subscriber that receives control raw setpoints via mavros */
            _pos_raw_sub = _nh.subscribe<mavros_msgs::PositionTarget>(
                "/" + _id + "/mavros/setpoint_raw/local", 5, &quad_class::command_callback, this);

            /** @brief Publisher that publishes odometry data */
            _odom_pub = _nh.advertise<nav_msgs::Odometry>(
                "/" + _id + "/mavros/odom_nwu", 10);
            /** @brief Publisher that publishes pose data */
            _pose_pub = _nh.advertise<geometry_msgs::PoseStamped>(
                "/" + _id + "/uav/nwu", 10);
            /** @brief Publisher that publishes common state data for tracker */
            _agent_pub = _nh.advertise<sensor_msgs::JointState>("/agent/pose", 10);
            /** @brief Publisher that publishes local pointcloud */
            _local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>(
                "/" + _id + "/map", 10);

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
            _drone_timer = _nh.createTimer(
                ros::Duration(_simulation_interval), 
                &quad_class::drone_timer, this, false, false);
            _map_timer = _nh.createTimer(
                ros::Duration(_map_interval), 
                &quad_class::map_timer, this, false, false);

            printf("[quad] %sdrone%d%s start_pose [%s%.2lf %.2lf %.2lf%s]! \n", 
                KGRN, uav_id, KNRM,
                KBLU, _start(0), _start(1), _start(2), KNRM);

            odom.pose.pose.position.x = _start(0);
            odom.pose.pose.position.y = _start(1);
            odom.pose.pose.position.z = _start(2);

            p_c_pos = f_c_pos = _start;
            p_c_vel = f_c_vel = Eigen::Vector3d::Zero();
            p_c_acc = f_c_acc = Eigen::Vector3d::Zero();

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

            _desired.pos = Eigen::Vector3d(_start(0), _start(1), _start(2));
            _desired.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
            _desired.acc = Eigen::Vector3d(0.0, 0.0, 0.0);
            _desired.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

            // Choose a color for the trajectory using random values
            std::random_device dev;
            std::mt19937 generator(dev());
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            color_vect = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);

            printf("[quad] %sdrone%d%s pcd path: %s\n", KGRN, uav_id, KNRM, _map_path.c_str());
            // try to load the file
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                _map_path, _global_map) == -1) 
            {
                printf("[quad] %sdrone%d%s no valid pcd used!\n", KRED, uav_id, KNRM);
            }

            _laser.set_parameters(
                _resolution,
                _sensing_range,
                _global_map,
                _vtc_laser_range_dgr,
                _hrz_laser_range_dgr,
                _vtc_laser_line_num,
                _hrz_laser_line_num);

            printf("[quad] %sdrone%d%s created! \n", KGRN, uav_id, KNRM);
            
            last_pose_time = ros::Time::now();

            _drone_timer.start();

            _map_timer.start();
        }

        ~quad_class()
        {
            _drone_timer.stop();
            
            _map_timer.stop();
            
            _global_map.points.clear();
        }

        bool check_sent_command(double tolerance);

        void drone_timer(const ros::TimerEvent &);

        void map_timer(const ros::TimerEvent &);

        void update_odom();

        void stop_and_hover();

        /** @brief callbacks */
        void command_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
        void trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

        /** @brief visualization functions */
        void visualize_uav();
        void visualize_log_path();
        void visualize_traj_path();

        /** @brief common utility functions */
        std::vector<quad_class::state> joint_trajectory_to_state(
            trajectory_msgs::JointTrajectory jt);

        Eigen::Quaterniond calc_uav_orientation(
            Eigen::Vector3d acc, double yaw_rad);

        void display_marker_list(
            std::vector<Eigen::Vector3d> vect, double scale,
            Eigen::Vector4d color, int id);

        geometry_msgs::PoseStamped eigen_to_posestamped(
            Eigen::Vector3d pos, Eigen::Quaterniond quat);

};