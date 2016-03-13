//reference: Nonlinear Ego-Motion Estimation from Optical Flow for Online Control of a Quadrotor UAV, Volker Grabe 

#include "wireless_localization.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <deque>
#include <vector>
#include "conversion.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace Eigen;

ros::Subscriber transmitter_sub;
ros::Publisher pose_pub;
vector<Vector3d> transmitters;
bool transmitters_initialized = false;

Matrix3d R_wireless2world;
bool wireless2world_initialized = false;

ros::Time now;//current time

void transmitterCallback(const sensor_msgs::PointCloudConstPtr &msg)
{
	for(int i = 0; i < msg->points.size(); i++)
	{
		Vector3d p;
		p(0) = msg->points[i].x;
		p(1) = msg->points[i].y;
		p(2) = msg->points[i].z;
		transmitters.push_back(p);
	}
	transmitters_initialized = true;
	transmitter_sub.shutdown();
}


// void AoACallback(const sensor_msgs::PointCloudConstPtr& msg)
void wirelessCallback(const sensor_msgs::PointCloudConstPtr& AoA_msg, const sensor_msgs::PointCloudConstPtr& rss_msg)
{
	if(! (transmitters_initialized && wireless2world_initialized) )  return;
	if(AoA_msg->points.size() != transmitters.size() || rss_msg->points.size() != transmitters.size()) 
	{
		cout << "rss&& AoA size and transmitters size is not equal" << endl;
		return;
	}

	vector<Vector3d> AoA;
	for(int i = 0; i < AoA_msg->points.size(); i++)
	{
		Vector3d p;
		p(0) = AoA_msg->points[i].x;
		p(1) = AoA_msg->points[i].y;
		p(2) = AoA_msg->points[i].z;
		p = R_wireless2world*p;//wireless frame to world frame
		AoA.push_back(p);
	}
	vector<double> rss;
	for(int i = 0; i < rss_msg->points.size(); i++)
	{
		double dis = rss_msg->points[i].x;
		rss.push_back(dis);
	}
	cout << "R: " << R_wireless2world << endl;
	Vector3d position = wireless_localize(transmitters, rss, AoA);
	cout << "position: " << position.transpose() << endl;
	//publish postion
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/world";
	pose.header.stamp = rss_msg->header.stamp;
	pose.pose.position.x = position(0);
	pose.pose.position.y = position(1);
	pose.pose.position.z = position(2);
	pose.pose.orientation.w = 1;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose_pub.publish(pose);
	now = rss_msg->header.stamp;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rss_localization");
	ros::NodeHandle n("~");
	string calibFile;

	transmitter_sub = n.subscribe("transmitter", 1000, transmitterCallback);
	//ros::Subscriber AoA_sub = n.subscribe("AoA", 1000, AoACallback);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/est_positon", 100);
	ros::Rate loop_rate(100);
	tf::TransformListener listener;


	message_filters::Subscriber<sensor_msgs::PointCloud> AoA_sub(n, "AoA", 100);
	message_filters::Subscriber<sensor_msgs::PointCloud> rss_sub(n, "rss", 100);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::PointCloud> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), AoA_sub, rss_sub);
	sync.registerCallback(boost::bind(&wirelessCallback, _1, _2));

	while(ros::ok())
	{
		ros::spinOnce();

		tf::StampedTransform transform;
		try{
		//targe_frame <- source frame
			
			listener.waitForTransform("/world", "wireless_receiver_link", 
					   now, ros::Duration(3.0));
			listener.lookupTransform("/world", "/wireless_receiver_link",
		               now, transform);
		}
			catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			loop_rate.sleep();
			continue;
		}
		Quaterniond qq;
		qq.w() = transform.getRotation().w();
		qq.x() = transform.getRotation().x();
		qq.y() = transform.getRotation().y();
		qq.z() = transform.getRotation().z();
		R_wireless2world = quaternion2mat(qq);
		wireless2world_initialized = true;
		loop_rate.sleep();
	}

	return 0;
}

