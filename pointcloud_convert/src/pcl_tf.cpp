#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>


class SegMapROSWraper  
{
private:
  ros::NodeHandle m_nh;  
  ros::Subscriber m_mavPoseSub;     //接收位姿
  ros::Subscriber m_pointCloudSub;  //接收点云
  ros::Publisher m_globalcloudPub;  //发布局部地图点云
  tf::StampedTransform m_sensorToWorldTf;   //定义存放变换关系的变量

public:
  SegMapROSWraper()
      : m_nh("~")  
  {
      m_mavPoseSub = m_nh.subscribe("/mavros/local_position/pose", 1, &SegMapROSWraper::mavPoseCallback, this);
      m_pointCloudSub = m_nh.subscribe("/rflysim/vehicle_lidar", 1, &SegMapROSWraper::vehicleCloudCallback, this);    //接收rosbag中的点云消息
      m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/map_server/local_map", 2, true);   //发布全局地图，用于rviz展示
  }

  ~SegMapROSWraper()
  {
      // delete m_pointCloudSub;
      // delete m_tfPointCloudSub;
  }
  

  void mavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &data)
  {
      tf::Transform transform;
      transform.setRotation(
            tf::Quaternion(
                    data->pose.orientation.x,
                    data->pose.orientation.y,
                    data->pose.orientation.z,
                    data->pose.orientation.w));
      transform.setOrigin(
            tf::Vector3(
                    data->pose.position.x,
                    data->pose.position.y,
                    data->pose.position.z));
      m_sensorToWorldTf = tf::StampedTransform(transform, ros::Time::now(), "world", "map");
  }

  void vehicleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
  {
      pcl::PointCloud<pcl::PointXYZ> pc;
      pcl::PointCloud<pcl::PointXYZ> pc_global;
      pcl::fromROSMsg(*cloud, pc);

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix(m_sensorToWorldTf, sensorToWorld);   //直接得到矩阵
      pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
      // std::cout<< sensorToWorld <<std::endl;
      sensor_msgs::PointCloud2 map_cloud;
      pcl::toROSMsg(pc_global, map_cloud);  //搞成消息
      map_cloud.header.stamp = ros::Time::now();
      map_cloud.header.frame_id = "map"; 
      m_globalcloudPub .publish(map_cloud);  //加上时间戳和frameid发布出来
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "pcl_tf"); 

  SegMapROSWraper  SM;

  ros::spin();
  return 0;
}

