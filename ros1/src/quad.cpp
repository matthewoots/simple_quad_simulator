#include <quad.h>

void quad_class::command_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{	
	std::lock_guard<std::mutex> cmd_lock(command_mutex);

	command_time = ros::Time::now();

	_desired.pos = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
	_desired.vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
	_desired.acc = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
	_desired.q = calc_uav_orientation(_desired.acc, msg->yaw);
	_desired.yaw = msg->yaw;

	geometry_msgs::PointStamped target;
	target.header.stamp = ros::Time::now();
	target.header.frame_id = "world";
	target.point.x = _desired.pos[0];
	target.point.y = _desired.pos[1];
	target.point.z = _desired.pos[2];
	_cmd_pub.publish(target);

	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;

	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = 0.0;
}

void quad_class::trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
	std::lock_guard<std::mutex> traj_lock(trajectory_mutex);

	trajectory_msgs::JointTrajectory copy_msg = *msg;
	std::string traj_agent_id = copy_msg.joint_names[0];
	if (traj_agent_id.compare(_id) != 0)
		return;
	traj_vector.clear();
	traj_pos_vector.clear();
	traj_vector = joint_trajectory_to_state(copy_msg);
}

bool quad_class::check_sent_command(double tolerance)
{
	// check_sent_command is true if command is within the tolerance
	std::lock_guard<std::mutex> cmd_lock(command_mutex);
	ros::Duration duration = ros::Time::now() - command_time;
	return duration.toSec() < tolerance;
}

void quad_class::drone_timer(const ros::TimerEvent &)
{
	// when UAV have not received any commands, UAV will hover
	if (!check_sent_command(_timeout))
	{
		if (mavros_custom_mode.compare("AUTO.LOITER")!=0)
		{
			mavros_custom_mode = "AUTO.LOITER";
			printf("%sdrone%d%s mode switch to %s%s%s! \n", 
				KGRN, uav_id, KNRM, 
				KBLU, mavros_custom_mode.c_str(), KNRM);
		}
		stop_and_hover();
		update_odom();
	}
	else
	{
		if (mavros_custom_mode.compare("AUTO.MISSION")!=0)
		{
			mavros_custom_mode = "AUTO.MISSION";
			printf("%sdrone%d%s mode switch to %s%s%s! \n", 
				KGRN, uav_id, KNRM, 
				KBLU, mavros_custom_mode.c_str(), KNRM);
		}
		update_odom();
	}

	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.stamp = ros::Time::now();
	pose_msg.header.frame_id = "world";
	pose_msg.pose = odom.pose.pose;

	_odom_pub.publish(odom);

	if ((ros::Time::now() - last_pose_time).toSec() > (1/_pose_pub_rate))
	{
		_pose_pub.publish(pose_msg);
		last_pose_time = ros::Time::now();
	}

	/** @brief For visualization */
	visualize_uav();
	visualize_log_path();
	visualize_traj_path();
}

void quad_class::map_timer(const ros::TimerEvent &)
{
	// ros::Time start = ros::Time::now();

	odometry_mutex.lock();
	nav_msgs::Odometry odom_copy = odom;
	odometry_mutex.unlock();

	Eigen::Quaterniond q;
	q.x() = odom_copy.pose.pose.orientation.x;
	q.y() = odom_copy.pose.pose.orientation.y;
	q.z() = odom_copy.pose.pose.orientation.z;
	q.w() = odom_copy.pose.pose.orientation.w;
	Eigen::Matrix3d rot(q);
	Eigen::Vector3d pos(
		odom_copy.pose.pose.position.x, 
		odom_copy.pose.pose.position.y, 
		odom_copy.pose.pose.position.z);

	pcl::PointCloud<pcl::PointXYZ> depth_map;      
	_laser.render_sensed_points(pos, rot, depth_map);

	sensor_msgs::PointCloud2 local_map_msg;
    pcl::toROSMsg(depth_map, local_map_msg);
    local_map_msg.header.frame_id = "world";
    local_map_msg.header.stamp = ros::Time::now();

    _local_pcl_pub.publish(local_map_msg);
	
	// printf("[quad] %sdrone%d%s mapping time %.3lfms\n", 
	// 	KGRN, uav_id, KNRM, (ros::Time::now() - start).toSec() * 1000);
}

void quad_class::update_odom()
{
	std::lock_guard<std::mutex> cmd_lock(command_mutex);

	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "world";

	_previous.pos = Eigen::Vector3d(
		odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
	
	_previous.vel = Eigen::Vector3d(
		odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
	
	_previous.acc = Eigen::Vector3d(
		odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);
		
	// Assuming mass is 1.0
	// 0.5 * rho * vel^2 * Cd * crossA
	// drag coefficient 1.0, 1.225kg/m3 density of air
	// double drag = 0.5 * 1.225 * pow(odom_vel.norm(),2) * 1.0 * 5.0;

	_current.pos = _desired.pos;
	_current.vel = _desired.vel;
	_current.acc = _desired.acc;
	_current.yaw = _desired.yaw;

	Eigen::Quaterniond q = calc_uav_orientation(_current.acc, _current.yaw);
	
	odometry_mutex.lock();
	odom.pose.pose.position.x = _current.pos[0];
	odom.pose.pose.position.y = _current.pos[1];
	odom.pose.pose.position.z = _current.pos[2];

	odom.pose.pose.orientation.w = q.w();
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();

	odom.twist.twist.linear.x = _current.vel[0];
	odom.twist.twist.linear.y = _current.vel[1];
	odom.twist.twist.linear.z = _current.vel[2];

	odom.twist.twist.angular.x = _current.acc[0];
	odom.twist.twist.angular.y = _current.acc[1];
	odom.twist.twist.angular.z = _current.acc[2];
	odometry_mutex.unlock();
}

void quad_class::stop_and_hover()
{	
	std::lock_guard<std::mutex> cmd_lock(command_mutex);

	_current.vel = Eigen::Vector3d(0, 0, 0);
	_current.acc = Eigen::Vector3d(0, 0, 0);
	_current.q = calc_uav_orientation(_current.acc, _current.yaw);
}

void quad_class::visualize_uav()
{
	std::lock_guard<std::mutex> odom_lock(odometry_mutex);

	visualization_msgs::Marker mesh;

	// input mesh model
	mesh.header.frame_id = "world";
	mesh.header.stamp = odom.header.stamp;
	mesh.ns = "drone";
	mesh.id = uav_id;
	mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
	mesh.action = visualization_msgs::Marker::ADD;

	// Copy posestamped message
	mesh.pose = odom.pose.pose;

	mesh.scale.x = 0.4;
	mesh.scale.y = 0.4;
	mesh.scale.z = 0.4;
	mesh.color.a = 0.7;
	mesh.color.r = 0.5;
	mesh.color.g = 0.5;
	mesh.color.b = 0.5;
	mesh.mesh_resource = _mesh_resource;
	_mesh_pub.publish(mesh);
}

Eigen::Quaterniond quad_class::calc_uav_orientation(
	Eigen::Vector3d acc, double yaw_rad)
{
	Eigen::Vector3d alpha = acc + Eigen::Vector3d(0,0,9.81);
	Eigen::Vector3d xC(cos(yaw_rad), sin(yaw_rad), 0);
	Eigen::Vector3d yC(-sin(yaw_rad), cos(yaw_rad), 0);
	Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
	Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
	Eigen::Vector3d zB = xB.cross(yB);

	Eigen::Matrix3d R;
	R.col(0) = xB;
	R.col(1) = yB;
	R.col(2) = zB;

	Eigen::Quaterniond q(R);
	return q;
}

void quad_class::visualize_log_path()
{
	std::lock_guard<std::mutex> odom_lock(odometry_mutex);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "world";

	// Copy posestamped message
	pose.pose = odom.pose.pose;

	// Log the path on Rviz
	// _simulation_interval/X = Time before we add another point to the vector
	if ((pose.header.stamp - log_previous_time).toSec() > _simulation_interval/10)
	{
		path.header = pose.header;
		path.poses.push_back(pose);
		_log_path_pub.publish(path);
	}

	// We remove the size of the path after several instances
	// _simulation_rate * X = number of seconds before removing the front of the vector
	if ((int)path.poses.size() > _simulation_rate * 20)
	{
		path.poses.erase(path.poses.begin());
	}
}

void quad_class::visualize_traj_path()
{
	std::lock_guard<std::mutex> traj_lock(trajectory_mutex);
	display_marker_list(traj_pos_vector, 0.1, color_vect, uav_id);
}

std::vector<quad_class::state> quad_class::joint_trajectory_to_state(
	trajectory_msgs::JointTrajectory jt)
{
	std::vector<quad_class::state> t_vect;
	// Size of joint trajectory
	int size_of_vector = (int)jt.points.size();
	for (int i = 0; i < size_of_vector; i++)
	{
		quad_class::state s;
		if (!jt.points[i].positions.empty())
			s.pos = Eigen::Vector3d(jt.points[i].positions[0],
					jt.points[i].positions[1], 
					jt.points[i].positions[2]);
		if (!jt.points[i].velocities.empty())
			s.vel = Eigen::Vector3d(jt.points[i].velocities[0],
					jt.points[i].velocities[1], 
					jt.points[i].velocities[2]);
		if (!jt.points[i].accelerations.empty())
			s.acc = Eigen::Vector3d(jt.points[i].accelerations[0],
					jt.points[i].accelerations[1], 
					jt.points[i].accelerations[2]);
		s.t = (jt.points[i].time_from_start).toSec();

		t_vect.push_back(s);
		traj_pos_vector.push_back(s.pos);
	}

	return t_vect;
}

void quad_class::display_marker_list(
	std::vector<Eigen::Vector3d> vect, double scale,
	Eigen::Vector4d color, int id)
{
	visualization_msgs::Marker sphere, line_strip;
	sphere.header.frame_id = line_strip.header.frame_id = "world";
	sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
	sphere.type = visualization_msgs::Marker::SPHERE_LIST;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	sphere.action = line_strip.action = visualization_msgs::Marker::ADD;

	int max_point_id = id + 1000;
	sphere.id = id;
	line_strip.id = max_point_id;

	sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
	sphere.color.r = line_strip.color.r = color(0);
	sphere.color.g = line_strip.color.g = color(1);
	sphere.color.b = line_strip.color.b = color(2);
	sphere.color.a = line_strip.color.a = color(3);
	sphere.scale.x = scale;
	sphere.scale.y = scale;
	sphere.scale.z = scale;
	line_strip.scale.x = scale / 2;
	
	for (int i = 0; i < (int)vect.size(); i++)
	{
		geometry_msgs::Point pt;
		pt.x = vect[i](0);
		pt.y = vect[i](1);
		pt.z = vect[i](2);
		sphere.points.push_back(pt);
		line_strip.points.push_back(pt);
	}
	_log_traj_pub.publish(sphere);
	_log_traj_pub.publish(line_strip);
}

geometry_msgs::PoseStamped quad_class::eigen_to_posestamped(
	Eigen::Vector3d pos, Eigen::Quaterniond quat)
{
	geometry_msgs::PoseStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "world";

	msg.pose.position.x = pos.x();
	msg.pose.position.y = pos.y();
	msg.pose.position.z = pos.z();
	
	msg.pose.orientation.x = quat.x();
	msg.pose.orientation.y = quat.y();
	msg.pose.orientation.z = quat.z();
	msg.pose.orientation.w = quat.w();

	return msg;
}