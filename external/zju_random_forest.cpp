#include <iostream>
#include <ctime>
#include <random>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Eigen>

#include <ros/ros.h>

class random_forest
{
	public:

		random_forest(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
		{
			_nh.param<int>("polar_obs", _polar_obs_num, 1);
			_nh.param<int>("circle_obs", _circle_obs_num, 1);
			_nh.param<int>("seed", _seed, 1);

			_nh.param<std::string>("path", _path, "");

			_nh.param<double>("map/resolution", _resolution, 0.1);
			_nh.param<double>("map/min_distance", _min_dist, 1.0);
			_nh.param<double>("map/x_size", _x_size, 50.0);
			_nh.param<double>("map/y_size", _y_size, 50.0);
			_nh.param<double>("map/z_size", _z_size, 5.0);
		
			_nh.param<double>("obstacle/lower_rad", _w_l, 0.3);
			_nh.param<double>("obstacle/upper_rad", _w_h, 0.8);
			_nh.param<double>("obstacle/lower_hei", _h_l, 3.0);
			_nh.param<double>("obstacle/upper_hei", _h_h, 7.0);
			_nh.param<double>("obstacle/radius_l", radius_l_, 7.0);
			_nh.param<double>("obstacle/radius_h", radius_h_, 7.0);
			_nh.param<double>("obstacle/z_l", z_l_, 7.0);
			_nh.param<double>("obstacle/z_h", z_h_, 7.0);
			_nh.param<double>("obstacle/theta", theta_, 7.0);

			_x_l = -_x_size / 2.0;
			_x_h = +_x_size / 2.0;

			_y_l = -_y_size / 2.0;
			_y_h = +_y_size / 2.0;

			_polar_obs_num = std::min(_polar_obs_num, (int)_x_size * 10);
			_z_limit = _z_size;

			// unsigned int seed = rd();
			eng.seed(_seed);

			RandomMapGenerateCylinder();
		}

		~random_forest(){}

		pcl::PointCloud<pcl::PointXYZ> get_pcl() 
		{
			return cloud_map;
		}

		std::string get_directory() 
		{
			return _path;
		}

	private:

		void RandomMapGenerateCylinder()
		{
			pcl::PointXYZ pt_random;

			std::vector<Eigen::Vector2d> obs_position;

			rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
			rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
			rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
			rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);
			rand_inf = std::uniform_real_distribution<double>(0.5, 1.5);

			rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
			rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
			rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
			rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

			// generate polar obs
			for (int i = 0; i < _polar_obs_num && ros::ok(); i++)
			{
				double x, y, w, h, inf;
				x = rand_x(eng);
				y = rand_y(eng);
				w = rand_w(eng);
				inf = rand_inf(eng);

				bool flag_continue = false;
				for (auto p : obs_position)
				if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
				{
					i--;
					flag_continue = true;
					break;
				}
				if (flag_continue)
				continue;

				obs_position.push_back(Eigen::Vector2d(x, y));

				x = floor(x / _resolution) * _resolution + _resolution / 2.0;
				y = floor(y / _resolution) * _resolution + _resolution / 2.0;

				int widNum = ceil((w * inf) / _resolution);
				double radius = (w * inf) / 2;

				for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
				for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
				{
					h = rand_h(eng);
					int heiNum = ceil(h / _resolution);
					for (int t = -10; t < heiNum; t++)
					{
					double temp_x = x + (r + 0.5) * _resolution + 1e-2;
					double temp_y = y + (s + 0.5) * _resolution + 1e-2;
					double temp_z = (t + 0.5) * _resolution + 1e-2;
					if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
					{
						pt_random.x = temp_x;
						pt_random.y = temp_y;
						pt_random.z = temp_z;
						cloud_map.points.push_back(pt_random);
					}
					}
				}
			}

			// generate circle obs
			for (int i = 0; i < _circle_obs_num; ++i)
			{
				double x, y, z;
				x = rand_x(eng);
				y = rand_y(eng);
				z = rand_z_(eng);

				x = floor(x / _resolution) * _resolution + _resolution / 2.0;
				y = floor(y / _resolution) * _resolution + _resolution / 2.0;
				z = floor(z / _resolution) * _resolution + _resolution / 2.0;

				Eigen::Vector3d translate(x, y, z);

				double theta = rand_theta_(eng);
				Eigen::Matrix3d rotate;
				rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
					1;

				double radius1 = rand_radius_(eng);
				double radius2 = rand_radius2_(eng);

				// draw a circle centered at (x,y,z)
				Eigen::Vector3d cpt;
				for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
				{
					cpt(0) = 0.0;
					cpt(1) = radius1 * cos(angle);
					cpt(2) = radius2 * sin(angle);

					// inflate
					Eigen::Vector3d cpt_if;
					for (int ifx = -0; ifx <= 0; ++ifx)
						for (int ify = -0; ify <= 0; ++ify)
						for (int ifz = -0; ifz <= 0; ++ifz)
						{
							cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
														ifz * _resolution);
							cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
							pt_random.x = cpt_if(0);
							pt_random.y = cpt_if(1);
							pt_random.z = cpt_if(2);
							cloud_map.push_back(pt_random);
						}
				}
			}

			// for (double x = -1.5; x <= 1.5; x += 0.1)
			//   for (double y = -1.5; y <= 1.5; y += 0.1)
			//     for (double z = -0.5; z <= 3.0; z += 0.1)
			//     {
			//       pt_random.x = x;
			//       pt_random.y = y;
			//       pt_random.z = z;
			//       cloud_map.push_back(pt_random);
			//     }

			cloud_map.width = cloud_map.points.size();
			cloud_map.height = 1;
			cloud_map.is_dense = true;

			ROS_WARN("Finished generate random map ");

		}

		ros::NodeHandle _nh;

		std::random_device rd;
		std::default_random_engine eng;
		std::uniform_real_distribution<double> rand_x;
		std::uniform_real_distribution<double> rand_y;
		std::uniform_real_distribution<double> rand_w;
		std::uniform_real_distribution<double> rand_h;
		std::uniform_real_distribution<double> rand_inf;

		std::vector<double> _state;

		std::string _path;

		int _seed;

		int _polar_obs_num;
		double _x_size, _y_size, _z_size;
		double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
		double _z_limit, _sensing_range, _resolution, _pub_rate;
		double _min_dist;

		int _circle_obs_num;
		double radius_l_, radius_h_, z_l_, z_h_;
		double theta_;

		std::uniform_real_distribution<double> rand_radius_;
		std::uniform_real_distribution<double> rand_radius2_;
		std::uniform_real_distribution<double> rand_theta_;
		std::uniform_real_distribution<double> rand_z_;

		pcl::PointCloud<pcl::PointXYZ> cloud_map;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_generation");
	ros::NodeHandle _nh("~");

	random_forest rf(_nh);

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);
	
	strftime(buffer,sizeof(buffer),"%d-%m-%Y_%H:%M:%S",timeinfo);
	std::string get_time(buffer);

	printf("save to path: %s\n", 
		(rf.get_directory() + get_time + ".pcd").c_str());

	pcl::PointCloud<pcl::PointXYZ> cloud_map = rf.get_pcl();

	pcl::io::savePCDFileASCII(
		rf.get_directory() + get_time + ".pcd", rf.get_pcl());

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud_map.makeShared());
	while (!viewer.wasStopped())
	{

	}

	ros::shutdown();

	return 0;
}