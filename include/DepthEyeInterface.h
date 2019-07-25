#ifndef POINTCLOUD_DEPTHEYE_H
#define POINTCLOUD_DEPTHEYE_H
#include "CameraSystem.h"
#include "Common.h"

// Comunicacao ROS
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>
#include <tf2_ros/static_transform_broadcaster.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/conversions.h>

//Standard Libs
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <string>



/*
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
*/

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
		DepthEyeSystem(ros::NodeHandle nh_camera, ros::NodeHandle nh_private, std::string nodeName); // new
		virtual ~DepthEyeSystem(); // new
		void publishData(); // new



	private:
		//ROS parameters
		ros::NodeHandle nh_, nh_private_;
		std::string nodeName_;
		camera_info_manager::CameraInfoManager *cim_tof_ /*, *cim_rgb*/;
		image_transport::ImageTransport *it_;
		image_transport::CameraPublisher pub_amp_, pub_dis_	/*, pub_rgb*/;
		tf2_ros::StaticTransformBroadcaster pub_tf;
		geometry_msgs::TransformStamped transformStamped;
		ros::Publisher pub_xyz_;
		//ros::Subscriber sub_amp_, sub_dis_;
		//boost::shared_ptr<ReconfigureServer> reconfigure_server_; // Nao soube achar o erro dessa linha pois deu erro no reconfigure_server_?
		bool config_init_;

		boost::mutex connect_mutex_;

		// Variables needed for config
		uint8_t udpDataIpAddr_[6], udpControlOutIpAddr_[6],
		udpControlInIpAddr_[6], tcpDeviceIpAddr_[6];
		std::string uartPortName_, calibFileName_;

		sensor_msgs::PointCloud2Ptr _xyz;

		//BTA_Handle handle_;
		//BTA_Config config_;

		//void callback(bta_tof_driver::bta_tof_driverConfig &config, uint32_t level);

		/**
		* @brief Reads configuration from the server parameters
		*/
		//void parseConfig();

		/**
		* @brief Returns the size of the data based in BTA_DataFormat
		*/
		//size_t getDataSize(BTA_DataFormat dataFormat);

		/**
		* @brief Returns the data encoding flat based in BTA_DataFormat
		*/
		//std::string getDataType(BTA_DataFormat dataFormat);

		/**
		* @brief Gives the conversion value to meters from the BTA_Unit
		* @param unit
		* @return the value to multiply to the data.
		*/
	 //float getUnit2Meters(BTA_Unit unit);


		/*###################
		  # Old parameters #
			##################
		*/
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
