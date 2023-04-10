#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_mavros_data_publisher");
  ros::NodeHandle nh;

  // Create publishers for GPS, IMU, relative altitude, and PNG image
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("mavros/global_position/global", 1);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("mavros/imu/data", 1);
  ros::Publisher alt_pub = nh.advertise<std_msgs::Float64>("mavros/global_position/rel_alt", 1);
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 1);

  // Create messages for GPS, IMU, relative altitude, and PNG image
  sensor_msgs::NavSatFix gps_msg;
  gps_msg.header.frame_id = "map";
  gps_msg.latitude = 37.7749;
  gps_msg.longitude = -122.4194;
  gps_msg.altitude = 50;

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "base_link";
  imu_msg.orientation.w = 1.0;
  imu_msg.angular_velocity.x = 0.0;
  imu_msg.angular_velocity.y = 0.0;
  imu_msg.angular_velocity.z = 0.0;
  imu_msg.linear_acceleration.x = 0.0;
  imu_msg.linear_acceleration.y = 0.0;
  imu_msg.linear_acceleration.z = 9.81;

  std_msgs::Float64 alt_msg;
  alt_msg.data = 100.0;

  cv::Mat image = cv::imread("/root/catkin_ws/src/brooklyn_project/mavros_dummy_data/src/test_image.png", cv::IMREAD_UNCHANGED);
  cv::Mat image_rgba;
  cv::cvtColor(image, image_rgba, cv::COLOR_BGRA2RGBA);
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgba8", image).toImageMsg();

  // Publish messages at a fixed rate
  ros::Rate rate(10);
  while (ros::ok())
  {
    gps_pub.publish(gps_msg);
    imu_pub.publish(imu_msg);
    alt_pub.publish(alt_msg);
    image_pub.publish(image_msg);

    rate.sleep();
  }

  return 0;
}
