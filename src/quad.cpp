#include <quad.h>

void quad_class::command_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{	
	std::lock_guard<std::mutex> cmd_lock(command_mutex);

	command_time = ros::Time::now();

	sc.pos = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
	sc.vel = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
	sc.acc = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
	double yaw = msg->yaw;
	sc.q = calc_uav_orientation(sc.acc, yaw);
	last_yaw = yaw;

	geometry_msgs::PointStamped target;
	target.header.stamp = ros::Time::now();
	target.header.frame_id = "world";
	target.point.x = sc.pos[0];
	target.point.y = sc.pos[1];
	target.point.z = sc.pos[2];
	_cmd_pub.publish(target);
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

	_odom_pub.publish(odom);

	/** @brief For visualization */
	visualize_uav();
	visualize_log_path();
}

void quad_class::update_odom()
{
	std::lock_guard<std::mutex> cmd_lock(command_mutex);

	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "world";

	Eigen::Vector3d odom_pos = Eigen::Vector3d(
		odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
	
	Eigen::Vector3d odom_vel = Eigen::Vector3d(
		odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);

	// Assuming mass is 1.0

	// 0.5 * rho * vel^2 * Cd * crossA
	// drag coefficient 1.0, 1.225kg/m3 density of air
	double drag = 0.5 * 1.225 * pow(odom_vel.norm(),2) * 1.0 * 2.0;

	Eigen::Vector3d _pos_error = sc.pos - odom_pos;

	double desired_vel = _max_vel * _pos_error.norm();
	
	// Avoid nan errors due to small _pos_errors
	Eigen::Vector3d _vector;
	if (_pos_error.norm() < 0.0000001)
		_vector = Eigen::Vector3d(0,0,0);

	else
		_vector = _pos_error / _pos_error.norm();
	Eigen::Vector3d motion_vector;
	if (odom_vel.norm() < 0.0000001)
		motion_vector = Eigen::Vector3d(0,0,0);

	else
		motion_vector = odom_vel / odom_vel.norm();

	Eigen::Vector3d _vel_error = desired_vel * _vector - odom_vel;
	_accumulated_vel_error += _vel_error;
	Eigen::Vector3d pid_vel_error = _vel_error * _p_gain + 
		(_accumulated_vel_error * _simulation_interval) * _i_gain + 
		(_vel_error / _simulation_interval) * _d_gain;

	// get the drag_vel through finding
	// drag_acc which is the opposing force drag * -motion_vector
	Eigen::Vector3d drag_acc = drag * -motion_vector;
	Eigen::Vector3d drag_vel = odom_vel + drag_acc * _simulation_interval;

	Eigen::Vector3d c_vel = odom_vel + drag_vel + pid_vel_error;
	Eigen::Vector3d c_acc = (c_vel-odom_vel)/_simulation_interval;

	// Eigen::Vector3d c_acc = odom_acc + pid_acc_error - drag * _vector;
	// Eigen::Vector3d c_vel = odom_vel + c_acc * _simulation_interval;	
	Eigen::Vector3d c_pos = odom_pos + (c_vel + odom_vel) / 2 * _simulation_interval;

	Eigen::Quaterniond q = calc_uav_orientation(c_acc, last_yaw);
	
	odom.pose.pose.position.x = c_pos[0];
	odom.pose.pose.position.y = c_pos[1];
	odom.pose.pose.position.z = c_pos[2];

	odom.pose.pose.orientation.w = q.w();
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();

	odom.twist.twist.linear.x = c_vel[0];
	odom.twist.twist.linear.y = c_vel[1];
	odom.twist.twist.linear.z = c_vel[2];

	odom.twist.twist.angular.x = c_acc[0];
	odom.twist.twist.angular.y = c_acc[1];
	odom.twist.twist.angular.z = c_acc[2];
}

void quad_class::stop_and_hover()
{	
	std::lock_guard<std::mutex> cmd_lock(command_mutex);

	sc.vel = Eigen::Vector3d(0, 0, 0);
	sc.acc = Eigen::Vector3d(0, 0, 0);
	double yaw = last_yaw;
	sc.q = calc_uav_orientation(sc.acc, yaw);
}

void quad_class::visualize_uav()
{
	visualization_msgs::Marker meshROS;
	// Mesh model
	meshROS.header.frame_id = "world";
	meshROS.header.stamp = odom.header.stamp;
	meshROS.ns = "drone";
	meshROS.id = uav_id;
	meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
	meshROS.action = visualization_msgs::Marker::ADD;
	meshROS.pose.position.x = odom.pose.pose.position.x;
	meshROS.pose.position.y = odom.pose.pose.position.y;
	meshROS.pose.position.z = odom.pose.pose.position.z;
	Eigen::Quaterniond q;
	q.w() = odom.pose.pose.orientation.w;
	q.x() = odom.pose.pose.orientation.x;
	q.y() = odom.pose.pose.orientation.y;
	q.z() = odom.pose.pose.orientation.z;
	// colvec ypr = R_to_ypr(quaternion_to_R(q));
	// ypr(0) += rotate_yaw * PI / 180.0;
	// q = R_to_quaternion(ypr_to_R(ypr));
	// if (cross_config)
	// {
	// 	colvec ypr = R_to_ypr(quaternion_to_R(q));
	// 	ypr(0) += 45.0 * PI / 180.0;
	// 	q = R_to_quaternion(ypr_to_R(ypr));
	// }
	meshROS.pose.orientation.w = q.w();
	meshROS.pose.orientation.x = q.x();
	meshROS.pose.orientation.y = q.y();
	meshROS.pose.orientation.z = q.z();
	meshROS.scale.x = 0.4;
	meshROS.scale.y = 0.4;
	meshROS.scale.z = 0.4;
	meshROS.color.a = 1.0;
	meshROS.color.r = 0.5;
	meshROS.color.g = 0.5;
	meshROS.color.b = 0.5;
	meshROS.mesh_resource = _mesh_resource;
	_mesh_pub.publish(meshROS);
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
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "world";
	pose.pose.position.x = odom.pose.pose.position.x;
	pose.pose.position.y = odom.pose.pose.position.y;
	pose.pose.position.z = odom.pose.pose.position.z;
	pose.pose.orientation.w = odom.pose.pose.orientation.w;
	pose.pose.orientation.x = odom.pose.pose.orientation.x;
	pose.pose.orientation.y = odom.pose.pose.orientation.y;
	pose.pose.orientation.z = odom.pose.pose.orientation.z;

	// Log the path on Rviz
	// _simulation_interval/X = Time before we add another point to the vector
	if ((pose.header.stamp - log_previous_time).toSec() > _simulation_interval/10)
	{
		path.header = pose.header;
		path.poses.push_back(pose);
		_log_path_pub.publish(path);
	}

	// We remove the size of the path after several instances
	// _state_pub_rate * X = number of seconds before removing the front of the vector
	if ((int)path.poses.size() > _state_pub_rate * 10)
	{
		path.poses.erase(path.poses.begin());
	}
}