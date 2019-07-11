#ifndef POINTCLOUD_DEPTHEYE_H
#define POINTCLOUD_DEPTHEYE_H
#include "CameraSystem.h"
#include "Common.h"

// Comunicacao ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>
/*
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf/transform_listener.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
*/
//Standard Libs
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <string>

namespace PointCloud
{
	enum DEPTH_MODE{
		UNKNOWN_MODE = 0,
		STANDARD,
		PRICISTION
	};

	enum DEVICE_STATUS
	{
		UNKNOWN_STATUS = 0,
		INITIALIZED,
		CONNECTED,
		STARTED,
		STOP
	};

	struct FrameSize
	{
	  uint32_t width, height;
	};

	class DepthEyeSystem
	{
	public:
		DepthEyeSystem();
		DepthEyeSystem(int vid,int pid);
		DepthEyeSystem(int vid,int pid,std::string&  usbFd);
		void setMode(DEPTH_MODE mode);
		DEVICE_STATUS connect();
		bool isInitialiszed(DEPTH_MODE mode);
		bool enableFilterHDR();
		bool enableFilterFlyingPixel(int threshold);
		void registerPointCloudCallback(Voxel::DepthCamera::CallbackType f);
		void registerRawDataCallback(Voxel::DepthCamera::CallbackType f);
		bool start();
		void stop();
		void reset();
		bool disconnect();
		float  getFrameRate();
		float  getFOV();
		FrameSize  getRevolution();

		//ROS Functions
		//DepthEyeSystem(ros::NodeHandle nh_camera, ros::NodeHandle nh_private, std::string nodeName);


	private:
		Voxel::CameraSystem	  cameraSys_;
		Voxel::DepthCameraPtr depthCamera_;
		Voxel::DevicePtr 	  device_;
		DEPTH_MODE 			  mode_;
		DEVICE_STATUS 		  status_;

		Voxel::DepthCamera::CallbackType pontcloud_func_;
		Voxel::DepthCamera::CallbackType rawdata_func_;
	};
}
#endif
