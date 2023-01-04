#include <uav_ros_control/reference/CarrotReference.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carrot_reference_node");
  ros::NodeHandle                nh;
  ros::NodeHandle                nh_private("~");
  uav_reference::CarrotReference carrot_obj(nh, nh_private);
  ros::spin();
}
