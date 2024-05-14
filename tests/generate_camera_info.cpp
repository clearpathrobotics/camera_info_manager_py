/**
  Example of saving and loading a camera_info yaml using the C++ camera_info_manager
*/

#include <camera_info_manager/camera_info_manager.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

// The saving of calibration files is private, this is how
// that is bypassed.
// (from https://github.com/ros-industrial/human_tracker/blob/master/packages/camera_info_manager/tests/unit_test.cpp )
// issue SetCameraInfo service request
void set_calibration(rclcpp::Node node,
                     const sensor_msgs::msg::CameraInfo &calib)
{
  ros::ServiceClient client =
      node.serviceClient<sensor_msgs::srv::SetCameraInfo>("set_camera_info");
  sensor_msgs::srv::SetCameraInfo set_camera_info;
  set_camera_info.request.camera_info = calib;
  client.call(set_camera_info);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("generate_camera_info");
  camera_info_manager::CameraInfoManager cim(nh);

  rclcpp::spin(node);
  // Can't call this private function
  // cim.saveCalibrationFile(camera_info, "cpp_cim_test.yaml", "camera");
  // this works but this node needs to just provide the service
  if (false)
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sensor_msgs::msg::CameraInfo camera_info;
    set_calibration(nh, camera_info);

    std::string url;
    ros::param::get("~url", url);
    cim.validateURL(url);
    cim.loadCameraInfo(url);
    // cim.saveCalibrationFile("cpp_cim_test2.yaml");
  }
}
