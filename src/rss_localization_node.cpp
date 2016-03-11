//reference: Nonlinear Ego-Motion Estimation from Optical Flow for Online Control of a Quadrotor UAV, Volker Grabe 

#include "rss_localization.h"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <deque>
#include <vector>


using namespace std;
using namespace Eigen;

ros::Subscriber transmitter_sub;
ros::Publisher pose_pub;
vector<Vector3d> transmitters;
bool transmitters_initialized = false;

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

void rssCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
	if(!transmitters_initialized) return;
	if(msg->points.size() != transmitters.size() ) 
	{
		cout << "rss size and transmitters size is not equal" << endl;
		return;
	}

	vector<double> rss;
	for(int i = 0; i < msg->points.size(); i++)
	{
		rss.push_back(msg->points[i].x);
	}
	Vector3d position = rss_localize(transmitters, rss);
	cout << "position: " << position.transpose() << endl;
	//publish postion
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/world";
	pose.header.stamp = msg->header.stamp;
	pose.pose.position.x = position(0);
	pose.pose.position.y = position(1);
	pose.pose.position.z = position(2);
	pose.pose.orientation.w = 1;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose_pub.publish(pose);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rss_localization");
	ros::NodeHandle n("~");

	transmitter_sub = n.subscribe("transmitter", 1000, transmitterCallback);
	ros::Subscriber rss_sub = n.subscribe("rss", 1000, rssCallback);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/est_positon", 100);
	ros::Rate loop_rate(100);
	ros::spin();
	
	return 0;
}

