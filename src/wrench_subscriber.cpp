#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include <Eigen/Dense>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
} 
Eigen::Matrix<double, 6, 1> force;
void wr_callback(const geometry_msgs::WrenchStamped &msg)
{
    force<<msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z;
    
}

int main(int argc, char **argv)
{
    /* code */
    ros::init(argc,argv,"receiver");

    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("test_controller/force_topic",1000,wr_callback);
     ros::Rate loop_rate(1);

    Eigen::Matrix<double,6,6> stiffness;
    double cartesian_stiff=200;
    double rotational_stiff=10;
    stiffness.setIdentity();
    stiffness.topLeftCorner(3,3)<<cartesian_stiff*Eigen::Matrix3d::Identity();
    stiffness.bottomRightCorner(3,3)<<rotational_stiff* Eigen::Matrix3d::Identity();
    while (ros::ok())
    {
      Eigen::Matrix<double,6,1> x_deformation;
      x_deformation<<stiffness.inverse()*force;
      ROS_INFO_STREAM("cartesian_position :"<< x_deformation);
      ros::spinOnce();
      loop_rate.sleep();
      /* code */
    }
    

    return 0;
}
