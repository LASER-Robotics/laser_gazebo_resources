#include "laser_gazebo_resources/livox_points_plugin.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>

namespace gazebo
{

  LivoxPointsPlugin::LivoxPointsPlugin() {}

  LivoxPointsPlugin::~LivoxPointsPlugin()
  {
    if (update_connection_)
    {
      update_connection_.reset();
    }
  }

  void LivoxPointsPlugin::convertDataToRotateInfo(const std::vector<std::vector<double>> &datas,
                                                  std::vector<AviaRotateInfo> &avia_infos)
  {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (size_t i = 0; i < datas.size(); ++i)
    {
      auto &data = datas[i];
      if (data.size() == 3)
      {
        avia_infos.emplace_back();
        avia_infos.back().time = data[0];
        avia_infos.back().azimuth = data[1] * deg_2_rad;
        avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;
        avia_infos.back().line = i % 4;
      }
      else
      {
        RCLCPP_WARN(ros_node_->get_logger(), "CSV data row size is not 3!");
      }
    }
  }

  void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {

    ros_node_ = gazebo_ros::Node::Get(_sdf);

    world_ = gazebo::physics::get_world(_sensor->WorldName());

    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_sensor);
    if (!sensor_)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Parent is not a ray sensor. Exiting");
      return;
    }

    // Load parameters
    // Load parameters
    std::vector<std::vector<double>> datas;
    std::string file_name_from_sdf = _sdf->Get<std::string>("csv_file_name");
    std::string absolute_csv_path;

    // Usa o ament_index para encontrar o caminho absoluto do arquivo CSV
    try
    {
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("laser_gazebo_resources");
      absolute_csv_path = package_share_directory + "/resources/" + file_name_from_sdf;
      RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Loading CSV file from: " << absolute_csv_path);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Package 'laser_gazebo_resources' not found. Cannot locate CSV file. Error: %s", e.what());
      return;
    }

    // Tenta ler o arquivo CSV usando o novo caminho absoluto
    if (!CsvReader::ReadCsvFile(absolute_csv_path, datas))
    {
      RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Cannot get CSV file: " << absolute_csv_path << ". Plugin will not run");
      return;
    }

    if (!_sdf->HasElement("robot_namespace"))
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Missing <robot_namespace>, defaults to \"uav1\"");
      robot_namespace_ = "uav1";
    }
    else
    {
      robot_namespace_ = _sdf->Get<std::string>("robot_namespace");
    }

    if (!_sdf->HasElement("parent_frame_name"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <parent_frame_name>, defaults to \"fcu\"");
      parent_frame_name_ = "fcu";
    }
    else
    {
      parent_frame_name_ = _sdf->Get<std::string>("parent_frame_name");
    }

    /* Sensor parameters //{ */

    if (!_sdf->HasElement("sensor_frame_name"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_frame_name>, defaults to \"livox\"");
      sensor_frame_name_ = "livox";
    }
    else
    {
      sensor_frame_name_ = _sdf->Get<std::string>("sensor_frame_name");
    }

    if (!_sdf->HasElement("sensor_x"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_x>, defaults to 0");
      sensor_x_ = 0.0;
    }
    else
    {
      sensor_x_ = _sdf->Get<double>("sensor_x");
    }

    if (!_sdf->HasElement("sensor_y"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_y>, defaults to 0");
      sensor_y_ = 0.0;
    }
    else
    {
      sensor_y_ = _sdf->Get<double>("sensor_y");
    }

    if (!_sdf->HasElement("sensor_z"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_z>, defaults to 0");
      sensor_z_ = 0.0;
    }
    else
    {
      sensor_z_ = _sdf->Get<double>("sensor_z");
    }

    if (!_sdf->HasElement("sensor_roll"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_roll>, defaults to 0");
      sensor_roll_ = 0.0;
    }
    else
    {
      sensor_roll_ = _sdf->Get<double>("sensor_roll");
    }

    if (!_sdf->HasElement("sensor_pitch"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_pitch>, defaults to 0");
      sensor_pitch_ = 0.0;
    }
    else
    {
      sensor_pitch_ = _sdf->Get<double>("sensor_pitch");
    }

    if (!_sdf->HasElement("sensor_yaw"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <sensor_yaw>, defaults to 0");
      sensor_yaw_ = 0.0;
    }
    else
    {
      sensor_yaw_ = _sdf->Get<double>("sensor_yaw");
    }

    //}

    /* Lidar parameters //{ */

    if (!_sdf->HasElement("lidar_frame_name"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_frame_name>, defaults to \"lidar\"");
      lidar_frame_name_ = "lidar";
    }
    else
    {
      lidar_frame_name_ = _sdf->Get<std::string>("lidar_frame_name");
    }

    if (!_sdf->HasElement("topic_lidar_name"))
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Missing <topic_lidar_name>, defaults to \"lidar\"");
      topic_lidar_name_ = "lidar";
    }
    else
    {
      topic_lidar_name_ = robot_namespace_ + "/" + _sdf->Get<std::string>("topic_lidar_name");
    }

    if (!_sdf->HasElement("lidar_x"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_x>, defaults to 0");
      lidar_x_ = 0.0;
    }
    else
    {
      lidar_x_ = _sdf->Get<double>("lidar_x");
    }

    if (!_sdf->HasElement("lidar_y"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_y>, defaults to 0");
      lidar_y_ = 0.0;
    }
    else
    {
      lidar_y_ = _sdf->Get<double>("lidar_y");
    }

    if (!_sdf->HasElement("lidar_z"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_z>, defaults to 0");
      lidar_z_ = 0.0;
    }
    else
    {
      lidar_z_ = _sdf->Get<double>("lidar_z");
    }

    if (!_sdf->HasElement("lidar_roll"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_roll>, defaults to 0");
      lidar_roll_ = 0.0;
    }
    else
    {
      lidar_roll_ = _sdf->Get<double>("lidar_roll");
    }

    if (!_sdf->HasElement("lidar_pitch"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_pitch>, defaults to 0");
      lidar_pitch_ = 0.0;
    }
    else
    {
      lidar_pitch_ = _sdf->Get<double>("lidar_pitch");
    }

    if (!_sdf->HasElement("lidar_yaw"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <lidar_yaw>, defaults to 0");
      lidar_yaw_ = 0.0;
    }
    else
    {
      lidar_yaw_ = _sdf->Get<double>("lidar_yaw");
    }

    //}

    if (!_sdf->HasElement("imu_frame_name"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_frame_name>, defaults to \"imu\"");
      imu_frame_name_ = "imu";
    }
    else
    {
      imu_frame_name_ = _sdf->Get<std::string>("imu_frame_name");
    }

    if (!_sdf->HasElement("imu_x"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_x>, defaults to 0");
      imu_x_ = 0.0;
    }
    else
    {
      imu_x_ = _sdf->Get<double>("imu_x");
    }

    if (!_sdf->HasElement("imu_y"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_y>, defaults to 0");
      imu_y_ = 0.0;
    }
    else
    {
      imu_y_ = _sdf->Get<double>("imu_y");
    }

    if (!_sdf->HasElement("imu_z"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_z>, defaults to 0");
      imu_z_ = 0.0;
    }
    else
    {
      imu_z_ = _sdf->Get<double>("imu_z");
    }

    if (!_sdf->HasElement("imu_roll"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_roll>, defaults to 0");
      imu_roll_ = 0.0;
    }
    else
    {
      imu_roll_ = _sdf->Get<double>("imu_roll");
    }

    if (!_sdf->HasElement("imu_pitch"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_pitch>, defaults to 0");
      imu_pitch_ = 0.0;
    }
    else
    {
      imu_pitch_ = _sdf->Get<double>("imu_pitch");
    }

    if (!_sdf->HasElement("imu_yaw"))
    {
      RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Missing <imu_yaw>, defaults to 0");
      imu_yaw_ = 0.0;
    }
    else
    {
      imu_yaw_ = _sdf->Get<double>("imu_yaw");
    }

    // --- Load scan pattern from CSV file ---
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    maxPointSize = aviaInfos.size();

    // --- Create physics collision and ray shape ---
    parentEntity = world_->EntityByName(_sensor->ParentName());
    auto physics = world_->Physics();
    laserCollision = physics->CreateCollision("multiray", _sensor->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_sensor->Pose());
    laserCollision->SetInitialRelativePose(_sensor->Pose());
    ray_shape_.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(ray_shape_);

    samplesStep = _sdf->Get<int>("samples");
    downSample = _sdf->Get<int>("downsample");
    if (downSample < 1)
    {
      downSample = 1;
    }

    publishPointCloudType = _sdf->Get<int>("publish_pointcloud_type");

    // --- Initialize ray shape parameters ---
    auto rayElem = _sdf->GetElement("ray");
    auto rangeElem = rayElem->GetElement("range");
    min_range_ = rangeElem->Get<double>("min");
    max_range_ = rangeElem->Get<double>("max");

    ray_shape_->RayShapes().reserve(samplesStep / downSample);
    ray_shape_->Load(_sdf);
    ray_shape_->Init();

    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample)
    {
      int index = j % maxPointSize;
      auto &rotate_info = aviaInfos[index];
      ignition::math::Quaterniond ray;
      ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
      auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
      start_point = min_range_ * axis + offset.Pos() - min_range_ * axis;
      end_point = max_range_ * axis + offset.Pos();
      ray_shape_->AddRay(start_point, end_point);
    }

    switch (publishPointCloudType)
    {
    case SENSOR_MSG_POINT_CLOUD:
      publisher_point_cloud_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud>(topic_lidar_name_, 5);
      break;
    case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
    case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
      publisher_point_cloud2_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_lidar_name_, 5);
      break;
    case laser_gazebo_resources_CUSTOM_MSG:
      publisher_custom_msg_ = ros_node_->create_publisher<laser_gazebo_resources::msg::CustomMsg>(topic_lidar_name_, 5);
      break;
    default:
      break;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);
    createStaticTransforms();
    publisher_tf_ = ros_node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).transient_local());
    timer_tf_ = ros_node_->create_wall_timer(std::chrono::seconds(1), std::bind(&LivoxPointsPlugin::publishStaticTransforms, this));

    update_connection_ = sensor_->ConnectUpdated(
        std::bind(&LivoxPointsPlugin::OnNewLaserScans, this));

    RCLCPP_INFO(ros_node_->get_logger(), "LivoxPointsPlugin loaded successfully!");
  }

  void LivoxPointsPlugin::OnNewLaserScans()
  {
    if (ray_shape_)
    {
      std::vector<std::pair<int, AviaRotateInfo>> points_pair;

      InitializeRays(points_pair, ray_shape_);

      ray_shape_->Update();

      switch (publishPointCloudType)
      {
      case SENSOR_MSG_POINT_CLOUD:
        PublishPointCloud(points_pair);
        break;
      case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
        PublishPointCloud2XYZ(points_pair);
        break;
      case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
        PublishPointCloud2XYZRTLT(points_pair);
        break;
      case laser_gazebo_resources_CUSTOM_MSG:
        PublishLivoxROSDriverCustomMsg(points_pair);
        break;
      default:
        break;
      }
    }
  }

  void LivoxPointsPlugin::createStaticTransforms()
  {
    tf2::Quaternion quat;
    rclcpp::Time stamp = ros_node_->get_clock()->now();

    // FCU to sensor
    geometry_msgs::msg::TransformStamped fcu_to_sensor_static_transform;
    fcu_to_sensor_static_transform.header.stamp = stamp;
    fcu_to_sensor_static_transform.header.frame_id = robot_namespace_ + "/" + parent_frame_name_;
    fcu_to_sensor_static_transform.child_frame_id = robot_namespace_ + "/" + sensor_frame_name_;
    fcu_to_sensor_static_transform.transform.translation.x = sensor_x_;
    fcu_to_sensor_static_transform.transform.translation.y = sensor_y_;
    fcu_to_sensor_static_transform.transform.translation.z = sensor_z_;

    quat.setRPY(sensor_roll_, sensor_pitch_, sensor_yaw_);
    fcu_to_sensor_static_transform.transform.rotation.x = quat.x();
    fcu_to_sensor_static_transform.transform.rotation.y = quat.y();
    fcu_to_sensor_static_transform.transform.rotation.z = quat.z();
    fcu_to_sensor_static_transform.transform.rotation.w = quat.w();

    tf_message_.transforms.push_back(fcu_to_sensor_static_transform);

    // Sensor to lidar
    geometry_msgs::msg::TransformStamped sensor_to_lidar_static_transform;
    sensor_to_lidar_static_transform.header.stamp = stamp;
    sensor_to_lidar_static_transform.header.frame_id = robot_namespace_ + "/" + sensor_frame_name_;
    sensor_to_lidar_static_transform.child_frame_id = robot_namespace_ + "/" + lidar_frame_name_;
    sensor_to_lidar_static_transform.transform.translation.x = lidar_x_;
    sensor_to_lidar_static_transform.transform.translation.y = lidar_y_;
    sensor_to_lidar_static_transform.transform.translation.z = lidar_z_;

    quat.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
    sensor_to_lidar_static_transform.transform.rotation.x = quat.x();
    sensor_to_lidar_static_transform.transform.rotation.y = quat.y();
    sensor_to_lidar_static_transform.transform.rotation.z = quat.z();
    sensor_to_lidar_static_transform.transform.rotation.w = quat.w();

    tf_message_.transforms.push_back(sensor_to_lidar_static_transform);

    // Sensor to IMU
    geometry_msgs::msg::TransformStamped sensor_to_imu_static_transform;
    sensor_to_imu_static_transform.header.stamp = stamp;
    sensor_to_imu_static_transform.header.frame_id = robot_namespace_ + "/" + sensor_frame_name_;
    sensor_to_imu_static_transform.child_frame_id = robot_namespace_ + "/" + imu_frame_name_;
    sensor_to_imu_static_transform.transform.translation.x = imu_x_;
    sensor_to_imu_static_transform.transform.translation.y = imu_y_;
    sensor_to_imu_static_transform.transform.translation.z = imu_z_;

    quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_);
    sensor_to_imu_static_transform.transform.rotation.x = quat.x();
    sensor_to_imu_static_transform.transform.rotation.y = quat.y();
    sensor_to_imu_static_transform.transform.rotation.z = quat.z();
    sensor_to_imu_static_transform.transform.rotation.w = quat.w();

    tf_message_.transforms.push_back(sensor_to_imu_static_transform);
  }

  void LivoxPointsPlugin::publishStaticTransforms()
  {
    publisher_tf_->publish(tf_message_);
  }

  void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                         boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
  {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.clear();
    points_pair.reserve(rays.size());
    for (int k = currStartIndex; k < end_index; k += downSample)
    {
      auto index = k % maxPointSize;
      auto &rotate_info = aviaInfos[index];
      ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
      auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
      start_point = min_range_ * axis + offset.Pos() - min_range_ * axis;
      end_point = max_range_ * axis + offset.Pos();
      if (ray_index < ray_size)
      {
        rays[ray_index]->SetPoints(start_point, end_point);
        points_pair.emplace_back(ray_index, rotate_info);
      }
      ray_index++;
    }
    currStartIndex += samplesStep;
    if (currStartIndex > maxPointSize)
    {
      currStartIndex -= maxPointSize;
    }
  }

  ignition::math::Angle LivoxPointsPlugin::AngleMin() const
  {
    if (ray_shape_)
      return ray_shape_->MinAngle();
    else
      return -1;
  }

  ignition::math::Angle LivoxPointsPlugin::AngleMax() const
  {
    if (ray_shape_)
    {
      return ignition::math::Angle(ray_shape_->MaxAngle().Radian());
    }
    else
      return -1;
  }

  double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

  double LivoxPointsPlugin::RangeMin() const
  {
    if (ray_shape_)
      return ray_shape_->GetMinRange();
    else
      return -1;
  }

  double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

  double LivoxPointsPlugin::RangeMax() const
  {
    if (ray_shape_)
      return ray_shape_->GetMaxRange();
    else
      return -1;
  }

  double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

  double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

  double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

  double LivoxPointsPlugin::RangeResolution() const
  {
    if (ray_shape_)
      return ray_shape_->GetResRange();
    else
      return -1;
  }

  int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

  int LivoxPointsPlugin::RayCount() const
  {
    if (ray_shape_)
      return ray_shape_->GetSampleCount();
    else
      return -1;
  }

  int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

  int LivoxPointsPlugin::RangeCount() const
  {
    if (ray_shape_)
      return ray_shape_->GetSampleCount() * ray_shape_->GetScanResolution();
    else
      return -1;
  }

  int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

  int LivoxPointsPlugin::VerticalRayCount() const
  {
    if (ray_shape_)
      return ray_shape_->GetVerticalSampleCount();
    else
      return -1;
  }

  int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

  int LivoxPointsPlugin::VerticalRangeCount() const
  {
    if (ray_shape_)
      return ray_shape_->GetVerticalSampleCount() * ray_shape_->GetVerticalScanResolution();
    else
      return -1;
  }

  ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
  {
    if (ray_shape_)
    {
      return ignition::math::Angle(ray_shape_->VerticalMinAngle().Radian());
    }
    else
      return -1;
  }

  ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
  {
    if (ray_shape_)
    {
      return ignition::math::Angle(ray_shape_->VerticalMaxAngle().Radian());
    }
    else
      return -1;
  }

  double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

  double LivoxPointsPlugin::VerticalAngleResolution() const
  {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
  }

  void LivoxPointsPlugin::PublishPointCloud(std::vector<std::pair<int, AviaRotateInfo>> &points_pair)
  {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    sensor_msgs::msg::PointCloud scan_point;
    scan_point.header.stamp = ros_node_->get_clock()->now();
    scan_point.header.frame_id = robot_namespace_ + "/" + lidar_frame_name_;
    auto &scan_points = scan_point.points;
    for (auto &pair : points_pair)
    {
      int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
      int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
      if (verticle_index < 0 || horizon_index < 0)
      {
        continue;
      }
      if (verticle_index < verticalRayCount && horizon_index < rayCount)
      {
        auto range = ray_shape_->GetRange(pair.first);
        auto intensity = ray_shape_->GetRetro(pair.first);
        if ((range >= max_range_) || (range <= min_range_))
        {
          continue;
        }

        auto rotate_info = pair.second;
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        auto point = range * axis;
        scan_points.emplace_back();
        scan_points.back().x = point.X();
        scan_points.back().y = point.Y();
        scan_points.back().z = point.Z();
      }
    }

    publisher_point_cloud_->publish(scan_point);
  }

  void LivoxPointsPlugin::PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>> &points_pair)
  {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    sensor_msgs::msg::PointCloud2 scan_point;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pc.reserve(points_pair.size());

    rclcpp::Time timestamp = ros_node_->get_clock()->now();

    for (const auto &pair : points_pair)
    {
      int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
      int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
      if (verticle_index < 0 || horizon_index < 0)
      {
        continue;
      }
      if (verticle_index < verticalRayCount && horizon_index < rayCount)
      {
        auto range = ray_shape_->GetRange(pair.first);

        if ((range >= max_range_) || (range <= min_range_))
        {
          continue;
        }

        auto rotate_info = pair.second;
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        auto point = range * axis;

        pcl::PointXYZ pt;
        pt.x = point.X();
        pt.y = point.Y();
        pt.z = point.Z();

        pc.push_back(pt);
      }
    }

    pcl::toROSMsg(pc, scan_point);
    scan_point.header.stamp = timestamp;
    scan_point.header.frame_id = robot_namespace_ + "/" + lidar_frame_name_;
    publisher_point_cloud2_->publish(scan_point);
  }

  void LivoxPointsPlugin::PublishPointCloud2XYZRTLT(std::vector<std::pair<int, AviaRotateInfo>> &points_pair)
  {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    sensor_msgs::msg::PointCloud2 scan_point;

    pcl::PointCloud<pcl::LivoxPointXyzrtlt> pc;
    pc.points.reserve(points_pair.size());
    rclcpp::Time header_timestamp = ros_node_->get_clock()->now();
    auto header_timestamp_sec_nsec = header_timestamp.nanoseconds();

    for (int i = 0; i < points_pair.size(); ++i)
    {
      std::pair<int, AviaRotateInfo> &pair = points_pair[i];
      int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
      int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
      if (verticle_index < 0 || horizon_index < 0)
      {
        continue;
      }
      if (verticle_index < verticalRayCount && horizon_index < rayCount)
      {
        auto range = ray_shape_->GetRange(pair.first);
        auto intensity = ray_shape_->GetRetro(pair.first);
        if ((range >= max_range_) || (range <= min_range_))
        {
          continue;
        }

        auto rotate_info = pair.second;
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        auto point = range * axis;
        pcl::LivoxPointXyzrtlt pt;

        pt.x = point.X();
        pt.y = point.Y();
        pt.z = point.Z();
        pt.intensity = static_cast<float>(intensity);
        pt.tag = 0;
        pt.line = pair.second.line;
        pt.timestamp = static_cast<double>(1e9 / 200000 * i) + header_timestamp_sec_nsec;

        pc.push_back(std::move(pt));
      }
    }

    pcl::toROSMsg(pc, scan_point);
    scan_point.header.stamp = header_timestamp;
    scan_point.header.frame_id = robot_namespace_ + "/" + lidar_frame_name_;
    publisher_point_cloud2_->publish(scan_point);
  }

  void LivoxPointsPlugin::PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>> &points_pair)
  {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    sensor_msgs::msg::PointCloud2 scan_point;

    laser_gazebo_resources::msg::CustomMsg msg;
    msg.header.frame_id = robot_namespace_ + "/" + lidar_frame_name_;

    struct timespec tn;
    clock_gettime(CLOCK_REALTIME, &tn);

    msg.timebase = tn.tv_nsec;
    msg.header.stamp = ros_node_->get_clock()->now();
    rclcpp::Time timestamp = ros_node_->get_clock()->now();
    for (int i = 0; i < points_pair.size(); ++i)
    {
      std::pair<int, AviaRotateInfo> &pair = points_pair[i];
      int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
      int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
      if (verticle_index < 0 || horizon_index < 0)
      {
        continue;
      }
      if (verticle_index < verticalRayCount && horizon_index < rayCount)
      {
        auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
        auto range = ray_shape_->GetRange(pair.first);
        auto intensity = ray_shape_->GetRetro(pair.first);
        if ((range >= max_range_) || (range <= min_range_))
        {
          continue;
        }

        auto rotate_info = pair.second;
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
        //                auto point = range * axis + world_pose.Pos(); Convert to world coordinate system

        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        auto point = range * axis;
        // pt.intensity = static_cast<float>(intensity);
        laser_gazebo_resources::msg::CustomPoint pt;
        pt.x = point.X();
        pt.y = point.Y();
        pt.z = point.Z();
        pt.line = pair.second.line;
        pt.tag = 0x10;
        pt.reflectivity = 100;
        pt.offset_time = (1e9 / 200000 * i);
        msg.points.push_back(pt);
      }
    }

    msg.point_num = msg.points.size();
    publisher_custom_msg_->publish(msg);
  }

  // Register this plugin
  GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

} // namespace gazebo
