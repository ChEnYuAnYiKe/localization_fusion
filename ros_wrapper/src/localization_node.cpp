#include "localization_wrapper.h"
#include <imu_gps_localization/uwb.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char** argv) {

	// Initialize ros.
	ros::init(argc, argv, "imu_gps_localization");
	ros::NodeHandle nh;

	// Initialize localizer.
	LocalizationWrapper localizer(nh);

	ros::spin();
	return 1;
}