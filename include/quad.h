#include <string>
#include <mutex>
#include <math.h>
#include <random>
#include <cmath>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

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

        ros::NodeHandle _nh;

        ros::Subscriber _pos_raw_sub; 
        ros::Subscriber _log_traj_sub; 
        ros::Subscriber _full_pcl_sub;

        ros::Publisher _odom_pub, _local_pcl_pub, _agent_pub;
        ros::Publisher _mesh_pub, _cmd_pub, _log_path_pub, _log_traj_pub;

        ros::Timer _drone_timer;
        ros::Timer _map_timer;

        mavros_msgs::PositionTarget _cmd;

        double _init_x, _init_y, _init_z;
        double _sensing_range, _resolution;
        int _hrz_laser_line_num, _vtc_laser_line_num;
        double _hrz_resolution_rad, _vtc_resolution_rad, _vtc_laser_range_rad;
        double _half_vtc_resolution_and_half_range;
        double _half_hrz_range = M_PI;

        bool _received_cmd = false;
        bool lerp_update;
        int uav_id;
        int interval_div, interval_count;
        std::string _id; 
        std::string _mesh_resource;
        std::string uav_id_char;
        std::string _map_path;
        double _simulation_interval;
        double _command_interval; 
        double _map_interval;
        double _state_pub_rate; 
        double _command_pub_rate;
        double _map_pub_rate;
        double _timeout, _yaw_offset, last_yaw;
        Eigen::Vector3d _start;

        Eigen::Vector3d f_c_pos, f_c_vel, f_c_acc;
        Eigen::Vector3d p_c_pos, p_c_vel, p_c_acc;

        pcl::PointCloud<pcl::PointXYZ> global_map;
        pcl::PointCloud<pcl::PointXYZ> local_map;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_local_map;
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_distance;
        Eigen::MatrixXi idx_map;
        Eigen::MatrixXd dis_map;

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

        std::vector<quad_class::state_command> traj_vector;

        std::vector<Eigen::Vector3d> traj_pos_vector;

        nav_msgs::Odometry odom;

        nav_msgs::Odometry previous_odom;

        quad_class(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("agent/mesh_resource", _mesh_resource, "");
            _nh.param<std::string>("agent/id", _id, "");
            _nh.param<double>("agent/state_pub_rate", _state_pub_rate, -1.0);
            _nh.param<double>("agent/command_rate", _command_pub_rate, -1.0);
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
            double hrz_laser_range_dgr, vtc_laser_range_dgr;
            _nh.param<double>("map/horizontal/laser_range_dgr", hrz_laser_range_dgr, 0.0);
            _nh.param<double>("map/vertical/laser_range_dgr", vtc_laser_range_dgr, 0.0);

            _vtc_laser_range_rad = vtc_laser_range_dgr / 180.0 * M_PI;
            _vtc_resolution_rad = _vtc_laser_range_rad / (double)(_vtc_laser_line_num - 1);
            _half_vtc_resolution_and_half_range = (_vtc_laser_range_rad + _vtc_resolution_rad) / 2.0;

            _half_hrz_range = hrz_laser_range_dgr / 180.0 * M_PI / 2.0;
            _hrz_resolution_rad = 2 * M_PI / (double)_hrz_laser_line_num;

            idx_map = Eigen::MatrixXi::Constant(_hrz_laser_line_num, _vtc_laser_line_num, -1);
            dis_map = Eigen::MatrixXd::Constant(_hrz_laser_line_num, _vtc_laser_line_num, 9999.0);

            _simulation_interval = 1 / _state_pub_rate;
            _command_interval = 1 / _command_pub_rate;
            _map_interval = 1 / _map_pub_rate;

            std::string copy_id = _id; 
            uav_id_char = copy_id.erase(0,5); // removes first 5 character
            uav_id = std::stoi(uav_id_char);

            /** @brief Subscriber that receives control raw setpoints via mavros */
            _pos_raw_sub = _nh.subscribe<mavros_msgs::PositionTarget>(
                "/" + _id + "/mavros/setpoint_raw/local", 5, &quad_class::command_callback, this);
            _log_traj_sub = _nh.subscribe<trajectory_msgs::JointTrajectory>(
                "/trajectory/points", 10, &quad_class::trajectory_callback, this);
            /** @brief Subscriber that receives full pointcloud */
            // _full_pcl_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
            //     "/ext/cloud", 10,  &quad_class::pcl2_callback, this);

            /** @brief Publisher that publishes odometry data */
            _odom_pub = _nh.advertise<nav_msgs::Odometry>(
                "/" + _id + "/mavros/odom_nwu", 10);
            /** @brief Publisher that publishes common state data for tracker */
            _agent_pub = _nh.advertise<sensor_msgs::JointState>("/agent/pose", 10);
            /** @brief Publisher that publishes local pointcloud */
            _local_pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>(
                "/" + _id + "/map", 10);

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

            previous_odom.pose.pose.position.x = _start(0);
            previous_odom.pose.pose.position.y = _start(1);
            previous_odom.pose.pose.position.z = _start(2);

            p_c_pos = f_c_pos = _start;
            p_c_vel = f_c_vel = Eigen::Vector3d::Zero();
            p_c_acc = f_c_acc = Eigen::Vector3d::Zero();

            interval_div = std::max(1,(int)std::round(_state_pub_rate / _command_pub_rate));
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
            std::mt19937 generator(dev());
            std::uniform_real_distribution<double> dis(0.0, 1.0);
            color_vect = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);

            printf("[quad] %sdrone%d%s pcd path: %s\n", KGRN, uav_id, KNRM, _map_path.c_str());
            // try to load the file
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(
                _map_path, global_map) == -1) 
            {
                printf("[quad] %sdrone%d%s no valid pcd used!\n", KRED, uav_id, KNRM);
            }

            kdtree_local_map.setInputCloud(global_map.makeShared());

            printf("[quad] %sdrone%d%s created! \n", KGRN, uav_id, KNRM);
            
            _drone_timer.start();

            _map_timer.start();
        }

        ~quad_class()
        {
            _drone_timer.stop();
            
            _map_timer.stop();
            
            global_map.points.clear();
        }

        bool check_sent_command(double tolerance);

        void drone_timer(const ros::TimerEvent &);

        void map_timer(const ros::TimerEvent &);

        void update_odom();

        void stop_and_hover();

        void render_sensed_points(
            nav_msgs::Odometry &_odom);

        inline void idx_to_pt(int x, int y, double dis, Eigen::Vector3d &pt);

        inline bool pt_to_idx(Eigen::Vector2i &idx,
            const Eigen::Vector3d &pt,
            const Eigen::Vector3d &laser_t, 
            const Eigen::Matrix3d &laser_R);

        inline double line_intersect_plane(Eigen::Vector3d &intersection,
            const Eigen::Vector3d &line_p, const Eigen::Vector3d &line_dir,
            const Eigen::Vector3d &plane_p, const Eigen::Vector3d &plane_normal);

        /** @brief callbacks */
        void command_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
        void trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
        void pcl2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

        /** @brief visualization functions */
        void visualize_uav();
        void visualize_log_path();
        void visualize_traj_path();

        /** @brief common utility functions */
        std::vector<quad_class::state_command> joint_trajectory_to_state(
            trajectory_msgs::JointTrajectory jt);

        Eigen::Quaterniond calc_uav_orientation(
            Eigen::Vector3d acc, double yaw_rad);

        void display_marker_list(
            std::vector<Eigen::Vector3d> vect, double scale,
            Eigen::Vector4d color, int id);

        /** @brief Convert point cloud from ROS sensor message to pcl point ptr **/
        // pcl::PointCloud<pcl::PointXYZ>::Ptr 
        //     pcl2_converter(sensor_msgs::PointCloud2 _pc)
        // {
        //     pcl::PCLPointCloud2 pcl_pc2;
        //     pcl_conversions::toPCL(_pc, pcl_pc2);

        //     pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
        //     pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
            
        //     return tmp_cloud;
        // }

        /** @brief Filter/Crop point cloud with the dimensions given */
        // inline pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr_box_crop(
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, 
        //     Eigen::Vector3d centroid, Eigen::Vector3d dimension)
        // {   
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr output(
        //         new pcl::PointCloud<pcl::PointXYZ>);

        //     Eigen::Vector3d min = centroid - (dimension / 2);
        //     Eigen::Vector3d max = centroid + (dimension / 2);

        //     pcl::CropBox<pcl::PointXYZ> box_filter;
        //     box_filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0));
        //     box_filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0));

        //     box_filter.setInputCloud(_pc);
        //     box_filter.filter(*output);

        //     return output;
        // }
};