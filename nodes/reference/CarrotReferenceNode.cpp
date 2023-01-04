#include <uav_ros_control/reference/CarrotReference.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carrot_reference_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Initialize distance control object
  std::shared_ptr<uav_reference::CarrotReference> carrotRefObj{
    new uav_reference::CarrotReference(nh, nh_private)
  };

  uav_reference::runDefault(*carrotRefObj, nh);
}
