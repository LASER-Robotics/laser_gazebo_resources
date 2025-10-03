#ifndef LIVOX_POINTS_PLUGIN_H
#define LIVOX_POINTS_PLUGIN_H

#include <string>
#include <vector>
#include <utility>

#include <ignition/math.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "laser_gazebo_resources/csv_reader.hpp"
#include "laser_gazebo_resources/livox_ode_multiray_shape.h"
#include "laser_gazebo_resources/livox_point_xyzrtl.h"

#include "laser_gazebo_resources/msg/custom_msg.hpp"
#include "laser_gazebo_resources/msg/custom_point.hpp"

struct LivoxPointXyzrtlt
{
  PCL_ADD_POINT4D;
  float reflectivity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

namespace gazebo
{

  // AviaRotateInfo is a simple struct and can be in the gazebo namespace
  typedef struct
  {
    double time;
    double azimuth;
    double zenith;
    int line;
  } AviaRotateInfo;

  class LivoxPointsPlugin : public SensorPlugin
  {
  public:
    LivoxPointsPlugin();

    virtual ~LivoxPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  private:
    ignition::math::Angle AngleMin() const;

    ignition::math::Angle AngleMax() const;

    double GetAngleResolution() const GAZEBO_DEPRECATED(7.0);

    double AngleResolution() const;

    double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

    double RangeMin() const;

    double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

    double RangeMax() const;

    double GetRangeResolution() const GAZEBO_DEPRECATED(7.0);

    double RangeResolution() const;

    int GetRayCount() const GAZEBO_DEPRECATED(7.0);

    int RayCount() const;

    int GetRangeCount() const GAZEBO_DEPRECATED(7.0);

    int RangeCount() const;

    int GetVerticalRayCount() const GAZEBO_DEPRECATED(7.0);

    int VerticalRayCount() const;

    int GetVerticalRangeCount() const GAZEBO_DEPRECATED(7.0);

    int VerticalRangeCount() const;

    ignition::math::Angle VerticalAngleMin() const;

    ignition::math::Angle VerticalAngleMax() const;

    double GetVerticalAngleResolution() const GAZEBO_DEPRECATED(7.0);

    double VerticalAngleResolution() const;

  private:
    void OnNewLaserScans();
    void PublishPointCloud(std::vector<std::pair<int, AviaRotateInfo>> &points_pair);
    void PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>> &points_pair);
    void PublishPointCloud2XYZRTLT(std::vector<std::pair<int, AviaRotateInfo>> &points_pair);
    void PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>> &points_pair);
    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos);
    void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                        boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape);
    void createStaticTransforms();
    void publishStaticTransforms();

  private:
    std::shared_ptr<gazebo_ros::Node> ros_node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_point_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud2_;
    rclcpp::Publisher<laser_gazebo_resources::msg::CustomMsg>::SharedPtr publisher_custom_msg_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tf_;
    rclcpp::TimerBase::SharedPtr timer_tf_;

    gazebo::sensors::RaySensorPtr sensor_;

    gazebo::physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;

    sdf::ElementPtr sdfPtr;
    gazebo::physics::EntityPtr parentEntity;
    gazebo::physics::CollisionPtr laserCollision;
    boost::shared_ptr<gazebo::physics::LivoxOdeMultiRayShape> ray_shape_;

    std::string robot_namespace_;
    std::string parent_frame_name_;

    // Sensor parameters
    std::string sensor_frame_name_;
    double sensor_x_;
    double sensor_y_;
    double sensor_z_;
    double sensor_roll_;
    double sensor_pitch_;
    double sensor_yaw_;

    // Lidar parameters
    std::string lidar_frame_name_;
    std::string topic_lidar_name_;
    double lidar_x_;
    double lidar_y_;
    double lidar_z_;
    double lidar_roll_;
    double lidar_pitch_;
    double lidar_yaw_;

    // IMU parameters
    std::string imu_frame_name_;
    double imu_x_;
    double imu_y_;
    double imu_z_;
    double imu_roll_;
    double imu_pitch_;
    double imu_yaw_;

    tf2_msgs::msg::TFMessage tf_message_;

    enum PublishPointCloudType
    {
      SENSOR_MSG_POINT_CLOUD = 0,
      SENSOR_MSG_POINT_CLOUD2_POINTXYZ = 1,
      SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT = 2,
      laser_gazebo_resources_CUSTOM_MSG = 3
    };
    int publishPointCloudType;

    std::vector<AviaRotateInfo> aviaInfos;
    int currStartIndex;
    int maxPointSize;
    int samplesStep;
    int downSample;
    double max_range_;
    double min_range_;
  };

} // namespace gazebo

#endif // LIVOX_POINTS_PLUGIN_H
