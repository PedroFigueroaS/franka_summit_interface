#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/WrenchStamped.h"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

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

class platform_adm_control
{
private:
    Eigen::Matrix<double,6,1> force;
    ros::Publisher pub;
    ros::Subscriber sub;
    Eigen::Matrix<double,6,6> stiffness;
    Eigen::Matrix<double,6,1> x_deformation;

    /* data */
public:
    platform_adm_control(ros::NodeHandle *nh)
    {
        double cartesian_stiff=200;
        double rotational_stiff=10;
        stiffness.setIdentity();
        stiffness.topLeftCorner(3,3)<<cartesian_stiff*Eigen::Matrix3d::Identity();
        stiffness.bottomRightCorner(3,3)<<rotational_stiff* Eigen::Matrix3d::Identity();
        pub=nh->advertise<geometry_msgs::PoseStamped>("position",1000);
        sub=nh->subscribe("test_controller/force_topic",1000, &platform_adm_control::wrench_callback, this);
    }

    void wrench_callback(const geometry_msgs::WrenchStamped &msg)
    {
        force<<msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z;
        x_deformation<<stiffness.inverse()*force;
        ROS_INFO_STREAM("cartesian_position :"<< x_deformation);
        geometry_msgs::PoseStamped def;
        def.pose.position.x=x_deformation(0);
        def.pose.position.y=x_deformation(1);
        def.pose.position.z=x_deformation(2);
        pub.publish(def);
    }

};


int main(int argc, char **argv)
{
    ros::init(argc,argv,"pub_sub_wrench");
    ros::NodeHandle nh;
    platform_adm_control ob = platform_adm_control(&nh);
    ros::spin();
    return 0;
}
