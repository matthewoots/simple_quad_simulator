#include "quad.h"

/**
 * @brief
 * Given a line and a plane, calculating for the intersection and the distance.
 * line_dir is required to be a normalized vector.
 * @param intersection the intersecting point.
 * @param line_p A point in the line.
 * @param line_dir The line direction vector.
 * @param plane_p A point in the plane.
 * @param plane_normal The plane normal vector.
 * @return double
 * The distance between the query point and the intersection.
 * A negtive value means the line direction vector points away from the plane.
 */
inline double quad_class::line_intersect_plane(Eigen::Vector3d &intersection,
    const Eigen::Vector3d &line_p, const Eigen::Vector3d &line_dir,
    const Eigen::Vector3d &plane_p, const Eigen::Vector3d &plane_normal)
{
    double d = (plane_p - line_p).dot(plane_normal) / line_dir.dot(plane_normal);
    intersection = line_p + d * line_dir;
    return d;
}

/**
 * @brief
 * filter the points not in range
 * @param idx
 * @param pt
 * @param laser_t
 * @param laser_R must be normalized in each column
 * @return true
 * @return false
 */
inline bool quad_class::pt_to_idx(Eigen::Vector2i &idx,
    const Eigen::Vector3d &pt,
    const Eigen::Vector3d &laser_t, const Eigen::Matrix3d &laser_R)
{
    Eigen::Vector3d inter_p;
    double dis_pt_to_laser_plane = line_intersect_plane(inter_p, pt, laser_R.col(2), laser_t, laser_R.col(2));
    double dis_laser_to_inter_p = (inter_p - laser_t).norm();
    double vtc_rad = std::atan2((pt - laser_t).dot(laser_R.col(2)), dis_laser_to_inter_p);
    if (std::fabs(vtc_rad) >= _half_vtc_resolution_and_half_range)
        return false;

    double x_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(0));
    double y_in_roll_pitch_plane = (inter_p - laser_t).dot(laser_R.col(1));
    double hrz_rad = std::atan2(y_in_roll_pitch_plane, x_in_roll_pitch_plane);
    if (std::fabs(hrz_rad) >= _half_hrz_range)
        return false;

    vtc_rad += _half_vtc_resolution_and_half_range;
    int vtc_idx = std::floor(vtc_rad / _vtc_resolution_rad);
    if (vtc_idx >= _vtc_laser_line_num)
        vtc_idx = 0;

    hrz_rad += M_PI + _hrz_resolution_rad / 2.0;
    int hrz_idx = std::floor(hrz_rad / _hrz_resolution_rad);
    if (hrz_idx >= _hrz_laser_line_num)
        hrz_idx = 0;

    idx << hrz_idx, vtc_idx;
    return true;
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param dis
 * @param pt in laser coordinate.
 */
inline void quad_class::idx_to_pt(int x, int y, double dis, Eigen::Vector3d &pt)
{
    double vtc_rad = y * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
    double hrz_rad = x * _hrz_resolution_rad - M_PI;
    pt[2] = sin(vtc_rad) * dis; // z_in_laser_coor
    double xy_square_in_laser_coor = cos(vtc_rad) * dis;
    pt[0] = cos(hrz_rad) * xy_square_in_laser_coor; // x_in_laser_coor
    pt[1] = sin(hrz_rad) * xy_square_in_laser_coor; // y_in_laser_coor
}

void quad_class::render_sensed_points(
    nav_msgs::Odometry &_odom)
{
    Eigen::Quaterniond q;
	q.x() = _odom.pose.pose.orientation.x;
	q.y() = _odom.pose.pose.orientation.y;
	q.z() = _odom.pose.pose.orientation.z;
	q.w() = _odom.pose.pose.orientation.w;
	Eigen::Matrix3d rot(q);
	Eigen::Vector3d laser_t(
		_odom.pose.pose.position.x, 
		_odom.pose.pose.position.y, 
		_odom.pose.pose.position.z);

	point_idx_radius_search.clear();
	point_radius_squared_distance.clear();
	pcl::PointXYZ searchPoint(
		_odom.pose.pose.position.x, 
		_odom.pose.pose.position.y, 
		_odom.pose.pose.position.z);
	kdtree_local_map.radiusSearch(
		searchPoint, _sensing_range, point_idx_radius_search, 
		point_radius_squared_distance);

    Eigen::Affine3d world_to_camera = Eigen::Affine3d::Identity();
    world_to_camera.translation() = laser_t;
    world_to_camera.linear() = rot;

	idx_map.setConstant(-1);
	dis_map.setConstant(9999.0);

	pcl::PointXYZ pt;
	for (size_t i = 0; i < point_idx_radius_search.size(); ++i)
	{
		pt = global_map.points[point_idx_radius_search[i]];

		Eigen::Vector2i idx;
		bool in_range = pt_to_idx(idx, Eigen::Vector3d(pt.x, pt.y, pt.z), laser_t, rot);
		if (!in_range)
		    continue;
        
		Eigen::Vector3d pt_vec(pt.x - _odom.pose.pose.position.x, 
            pt.y - _odom.pose.pose.position.y, 
            pt.z - _odom.pose.pose.position.z);
		double dis_curr_pt = pt_vec.norm();

        double vtc_rad = idx[1] * _vtc_resolution_rad - _vtc_laser_range_rad / 2.0;
        double dis_to_z_axis = dis_curr_pt * std::cos(vtc_rad);
        double mesh_len_hrz = dis_to_z_axis * _hrz_resolution_rad;
        double mesh_len_vtc = dis_to_z_axis * _vtc_resolution_rad;
        int hrz_occ_grid_num = std::min((int)std::floor(_resolution / mesh_len_hrz), _hrz_laser_line_num);
        int vtc_occ_grid_num = std::min((int)std::floor(_resolution / mesh_len_vtc), _vtc_laser_line_num);
        // ROS_INFO_STREAM("hrz_occ_grid_num " << hrz_occ_grid_num / 2 << ", vtc_occ_grid_num " << vtc_occ_grid_num / 2);
        int tmp1 = hrz_occ_grid_num, tmp2 = vtc_occ_grid_num;
        for (int d_hrz_idx = -tmp1; d_hrz_idx <= tmp1; ++d_hrz_idx)
            for (int d_vtc_idx = -tmp2; d_vtc_idx <= tmp2; ++d_vtc_idx)
            {
                // it's a ring in hrz coordiante
                int hrz_idx = (idx[0] + d_hrz_idx + _hrz_laser_line_num) % _hrz_laser_line_num; 
                int vtc_idx = idx[1] + d_vtc_idx;
                if (vtc_idx >= _vtc_laser_line_num)
                    continue;

                if (vtc_idx < 0)
                    continue;

                if (dis_curr_pt < dis_map(hrz_idx, vtc_idx))
                {
                    idx_map(hrz_idx, vtc_idx) = i;
                    dis_map(hrz_idx, vtc_idx) = dis_curr_pt;
                }
            }
    }

    local_map.points.clear();
    for (int x = 0; x < _hrz_laser_line_num; ++x)
        for (int y = 0; y < _vtc_laser_line_num; ++y)
        {
            // use laser line pts
            Eigen::Vector3d p;
            if (idx_map(x, y) != -1)
            {
                idx_to_pt(x, y, dis_map(x, y), p);
                Eigen::Affine3d camera_to_point = 
                    Eigen::Affine3d::Identity();
                camera_to_point.translation() = p;

                Eigen::Vector3d world_point = 
                    (world_to_camera * camera_to_point).translation();

                local_map.points.emplace_back(
                    world_point.x(), world_point.y(), world_point.z());
            }
        }

    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

    sensor_msgs::PointCloud2 local_map_msg;
    pcl::toROSMsg(local_map, local_map_msg);
    local_map_msg.header.frame_id = "world";
    local_map_msg.header.stamp = ros::Time::now();

    _local_pcl_pub.publish(local_map_msg);
	
}