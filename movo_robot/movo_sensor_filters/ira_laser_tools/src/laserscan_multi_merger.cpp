//
// source: https://github.com/iralabdisco/ira_laser_tools
//

#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{
public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

private:
    struct ScanCloud {
		ScanCloud(ros::Subscriber sub):
			subscriber(sub),
			cloud(),
			cloud_modified(false) {}

		ros::Subscriber subscriber;
		pcl::PCLPointCloud2 cloud;
		bool cloud_modified;
	};
	
	ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Publisher laser_scan_publisher_;

	map<string, ScanCloud> scan_clouds;

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;
};

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void LaserscanMerger::laserscan_topic_parser()
{
	int attempts = 0;
	ros::master::V_TopicInfo published_topics;

    istringstream iss(laserscan_topics);
	unordered_set<string> tokens;
	for (istream_iterator<string> it(iss); i != istream_iterator<string>; ++it)
	{
		tokens.insert(*it);
	}

	ros::Rate retry_rate(0.5); // [hz]
	while (attempts < 10)
	{
		ros::mater::getTopics(published_topics);
		for (int i = 0; i < published_topics.size(); ++i)
		{
			if (tokens.find(published_topics[i].name) != tokens.end())
			{
				if (!published_topics[i].datatype.compare("sensor_msgs/LaserScan"))
					continue;

				string topic_name(published_topics[i].name);
				ROS_INFO("Laser topic %d: %s", i, topic_name.c_str());
				
				scan_clouds.insert(std::pair<string, ScanCloud>(
					topic_name,
					ScanCloud(node_.subscribe<sensor_msgs::LaserScan>(
						topic_name.c_str(),
						1,
						boost::bind(&LaserscanMerger::scanCallback,this, _1, topic_name)
					))
				));
			}
		}

		if (scan_clouds.size() == tokens.size())
		{
			ROS_INFO("Subscribing to %ld laser topics", scan_clouds.size());
			break;
		}
		else
		{
			ROS_WARN("Failed to find laser topics for merging....");
		}

		attempts++;
	    ROS_INFO("Laser topics not published yet; waiting 2 seconds and retrying....");
	    retry_rate.sleep();
	}
}

LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

	nh.getParam("destination_frame", destination_frame);
	nh.getParam("cloud_destination_topic", cloud_destination_topic);
	nh.getParam("scan_destination_topic", scan_destination_topic);
    nh.getParam("laserscan_topics", laserscan_topics);

    this->laserscan_topic_parser();

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);

	//tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));

	projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}

	unordered_set<string, ScanCloud> find_result = scan_clouds.find(topic);
	if (find_result != scan_clouds.end())
	{
		sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
		pcl_conversions::toPCL(tmpCloud3, find_result->clouds);
		find_result->cloud_modified = true;
	}	

    // Count how many scans we have
	int totalClouds = 0;
	for (unordered_set<string, ScanCloud>::iterator it = scan_clouds.begin(); it != scan_clouds.end(); ++it)
	{
		totalClouds += it->cloud_modified ? 1 : 0;
	}

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == scan_clouds.size())
	{
		pcl::PCLPointCloud2 merged_cloud;
		for (unordered_set<string, ScanCloud>::iterator it = scan_clouds.begin(); it != scan_clouds.end(); ++it)
		{
			pcl::concatenate(merged_cloud, it->cloud, merged_cloud);
			it->cloud_modified = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud, points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = ros::Time::now();  //fixes #265
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;


		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_.publish(output);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_multi_merger");

    LaserscanMerger _laser_merger;

    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
