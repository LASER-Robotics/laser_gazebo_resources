#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gazebo_ros/node.hpp>

#include "laser_gazebo_resources/common.h"
#include "laser_gazebo_resources/perlin_noise.h"

namespace gazebo
{
namespace
{

constexpr char kDepthCameraTopic[] = "aligned_depth_to_color";
constexpr char kColorCameraTopic[] = "color";
constexpr char kInfrared1CameraTopic[] = "infra1";
constexpr char kInfrared2CameraTopic[] = "infra2";

constexpr char kDepthCameraSuffix[] = "aligned_depth_to_color";
constexpr char kColorCameraSuffix[] = "color";
constexpr char kInfraredStereoCameraSuffix[] = "infra_stereo";
constexpr char kInfrared1CameraSuffix[] = "infra1";
constexpr char kInfrared2CameraSuffix[] = "infra2";
constexpr char kBaseFrameSuffix[] = "link";

constexpr unsigned int kDepthPublishFrequencyHz = 60U;
constexpr unsigned int kColorPublishFrequencyHz = 30U;
constexpr unsigned int kInfrared1PublishFrequencyHz = 60U;
constexpr unsigned int kInfrared2PublishFrequencyHz = 60U;

constexpr double kDepthScaleMeters = 0.001;
constexpr unsigned int kPointCloudDecimation = 4U;
constexpr double kHalfPi = 1.57079632679489661923;

}  // namespace

class RealSensePlugin : public ModelPlugin
{
public:
  RealSensePlugin() = default;
  ~RealSensePlugin() override = default;

  /// Configure the Gazebo sensors, transport publishers, and frame callbacks.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    auto * sensor_manager = sensors::SensorManager::Instance();

    const std::string camera_name =
      sdf->HasElement("camera_name") ? sdf->Get<std::string>("camera_name") : "rgbd";
    const std::string camera_suffix =
      sdf->HasElement("camera_suffix") ? sdf->Get<std::string>("camera_suffix") : "";
    const std::string ros_namespace =
      sdf->HasElement("namespace") ? sdf->Get<std::string>("namespace") : "";

    ConfigureSensorNames(ros_namespace, camera_name, camera_suffix);
    if (!ConfigureSensors(sensor_manager)) {
      return;
    }

    ConfigureRealisticDepth(sdf);
    AllocateDepthBuffers();
    ConfigureGazeboTransport();
    ConnectSensorCallbacks();
  }

  /// Forward a regular camera frame through Gazebo transport.
  virtual void OnNewFrame(
    const rendering::CameraPtr camera,
    const transport::PublisherPtr publisher)
  {
    msgs::ImageStamped message;
    msgs::Set(message.mutable_time(), world_->SimTime());

    auto * image = message.mutable_image();
    image->set_width(camera->ImageWidth());
    image->set_height(camera->ImageHeight());
    image->set_pixel_format(
      common::Image::ConvertPixelFormat(camera->ImageFormat()));
    image->set_step(camera->ImageWidth() * camera->ImageDepth());
    image->set_data(
      camera->ImageData(),
      camera->ImageDepth() * camera->ImageWidth() * camera->ImageHeight());

    publisher->Publish(message);
  }

  /// Convert the simulated floating-point depth image to a 16-bit depth image.
  virtual void OnNewDepthFrame(
    const rendering::CameraPtr camera,
    const transport::PublisherPtr publisher)
  {
    (void)camera;

    const unsigned int image_width = depth_camera_->ImageWidth();
    const unsigned int image_height = depth_camera_->ImageHeight();
    const unsigned int image_size = image_width * image_height;
    const float * depth_data = depth_camera_->DepthData();

    for (unsigned int index = 0; index < image_size; ++index) {
      const float depth = depth_data[index];
      if (IsDepthInvalid(depth)) {
        depth_map_[index] = 0U;
      } else {
        depth_map_[index] = depth / kDepthScaleMeters;
      }
    }

    PublishGazeboDepthImage(
      publisher, depth_map_.data(), image_width, image_height, image_size);
  }

  /// Apply the configured sensor artifacts before publishing the depth image.
  virtual void OnNewDepthFrameRealistic(
    const rendering::CameraPtr camera,
    const transport::PublisherPtr publisher)
  {
    (void)camera;

    const unsigned int image_width = depth_camera_->ImageWidth();
    const unsigned int image_height = depth_camera_->ImageHeight();
    const unsigned int image_size = image_width * image_height;
    const float * depth_data = depth_camera_->DepthData();

    // This value intentionally remains shared across callbacks, matching the
    // original noise evolution over time.
    static float current_perlin_z = 6.0F;
    current_perlin_z += perlin_empty_speed_;

    unsigned int x = 0U;
    unsigned int y = 0U;
    for (unsigned int index = 0; index < image_size; ++index) {
      const float relative_x =
        5.0F * static_cast<float>(x) / static_cast<float>(image_width);
      const float relative_y =
        5.0F * static_cast<float>(y) / static_cast<float>(image_height);
      const float depth = depth_data[index];

      const bool remove_pixel =
        IsDepthInvalid(depth) ||
        perlin_noise_.noise(relative_x, relative_y, current_perlin_z) >
        perlin_empty_threshold_;

      if (remove_pixel) {
        small_depth_map_[index] = 0U;
      } else {
        const float noise_scale =
          depth > noise_min_distance_ ? depth : noise_min_distance_;
        const float noise = noise_scale * normal_distribution_(random_generator_);
        float noisy_depth = noise + depth;

        if (
          noisy_depth < 0.0F ||
          noisy_depth > kDepthScaleMeters * std::numeric_limits<uint16_t>::max())
        {
          noisy_depth = 0.0F;
        }

        small_depth_map_[index] = noisy_depth / kDepthScaleMeters;
      }

      ++x;
      if (x >= image_width) {
        x = 0U;
        ++y;
      }
    }

    const unsigned int output_width = image_width * scaling_;
    const unsigned int output_height = image_height * scaling_;
    const unsigned int output_size = output_width * output_height;

    cv::Mat depth_image(
      static_cast<int>(image_height),
      static_cast<int>(image_width),
      CV_16UC1,
      small_depth_map_.data());

    UpscaleDepthMap(depth_image, depth_image, static_cast<int>(scaling_));
    ApplyDepthBlur(depth_image);
    ApplyDepthErosion(depth_image);

    std::memcpy(
      depth_map_.data(),
      depth_image.ptr(0),
      sizeof(uint16_t) * output_size);

    PublishGazeboDepthImage(
      publisher, depth_map_.data(), output_width, output_height, output_size);
  }

protected:
  /// Keep the world-update connection used by the original plugin.
  void OnUpdate()
  {
  }

  std::string depth_camera_plugin_name_;
  std::string color_camera_plugin_name_;
  std::string infrared_stereo_camera_plugin_name_;
  std::string infrared1_camera_plugin_name_;
  std::string infrared2_camera_plugin_name_;

  bool use_realistic_{false};
  unsigned int scaling_{4U};
  float noise_per_meter_{0.0F};
  float noise_min_distance_{0.0F};
  float perlin_empty_speed_{0.0F};
  float perlin_empty_threshold_{0.0F};
  unsigned int blur_size_{0U};
  unsigned int erosion_size_{0U};

  PerlinNoise perlin_noise_;
  std::default_random_engine random_generator_;
  std::normal_distribution<float> normal_distribution_;

  physics::ModelPtr model_{nullptr};
  physics::WorldPtr world_{nullptr};
  rendering::DepthCameraPtr depth_camera_{nullptr};
  rendering::CameraPtr color_camera_{nullptr};
  sensors::MultiCameraSensorPtr infrared_stereo_camera_{nullptr};
  std::vector<rendering::CameraPtr> infrared_cameras_;

  transport::NodePtr transport_node_{nullptr};
  std::vector<uint16_t> depth_map_;
  std::vector<uint16_t> small_depth_map_;

  transport::PublisherPtr depth_publisher_{nullptr};
  transport::PublisherPtr color_publisher_{nullptr};
  transport::PublisherPtr infrared1_publisher_{nullptr};
  transport::PublisherPtr infrared2_publisher_{nullptr};

  event::ConnectionPtr new_depth_frame_connection_{nullptr};
  event::ConnectionPtr new_infrared1_frame_connection_{nullptr};
  event::ConnectionPtr new_infrared2_frame_connection_{nullptr};
  event::ConnectionPtr new_color_frame_connection_{nullptr};
  event::ConnectionPtr update_connection_{nullptr};

private:
  static void UpscaleDepthMap(
    const cv::Mat & input,
    cv::Mat & output,
    int scale)
  {
    const unsigned int input_rows = input.rows;
    const unsigned int input_columns = input.cols;
    const unsigned int output_rows = input_rows * scale;
    const unsigned int output_columns = input_columns * scale;

    cv::Mat resized(
      static_cast<int>(output_rows),
      static_cast<int>(output_columns),
      CV_16UC1);

    const uint16_t * input_data = input.ptr<uint16_t>(0);
    uint16_t * output_pixel = resized.ptr<uint16_t>(0);

    for (unsigned int row = 0; row < output_rows; ++row) {
      for (unsigned int column = 0; column < output_columns; ++column) {
        *output_pixel =
          input_data[column / scale + (row / scale) * input_columns];
        ++output_pixel;
      }
    }

    output = resized;
  }

  void ConfigureSensorNames(
    const std::string & ros_namespace,
    const std::string & camera_name,
    const std::string & camera_suffix)
  {
    const std::string sensor_prefix =
      ros_namespace + "/" + camera_name + camera_suffix + "_";

    depth_camera_plugin_name_ = sensor_prefix + kDepthCameraSuffix;
    color_camera_plugin_name_ = sensor_prefix + kColorCameraSuffix;
    infrared_stereo_camera_plugin_name_ =
      sensor_prefix + kInfraredStereoCameraSuffix;
    infrared1_camera_plugin_name_ = sensor_prefix + kInfrared1CameraSuffix;
    infrared2_camera_plugin_name_ = sensor_prefix + kInfrared2CameraSuffix;
  }

  bool ConfigureSensors(sensors::SensorManager * sensor_manager)
  {
    const auto depth_sensor =
      sensor_manager->GetSensor(depth_camera_plugin_name_);
    const auto color_sensor =
      sensor_manager->GetSensor(color_camera_plugin_name_);
    const auto infrared_sensor =
      sensor_manager->GetSensor(infrared_stereo_camera_plugin_name_);

    if (!depth_sensor || !color_sensor || !infrared_sensor) {
      gzerr << "RealSensePlugin: One or more sensors not found!" << std::endl;
      return false;
    }

    // Keep the original sensor-type assumptions. The SDF configuration must
    // provide the expected Gazebo sensor types.
    depth_camera_ =
      std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
      depth_sensor)->DepthCamera();
    infrared_stereo_camera_ =
      std::dynamic_pointer_cast<sensors::MultiCameraSensor>(infrared_sensor);
    color_camera_ =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      color_sensor)->Camera();

    return true;
  }

  void ConfigureRealisticDepth(sdf::ElementPtr sdf)
  {
    getSdfParam(sdf, "useRealistic", use_realistic_, false);

    if (use_realistic_) {
      getSdfParam(sdf, "imageScaling", scaling_, 4U);
      getSdfParam(sdf, "noisePerMeter", noise_per_meter_, 0.2F);
      getSdfParam(sdf, "noiseMinDistance", noise_min_distance_, 4.0F);
      getSdfParam(sdf, "perlinEmptySpeed", perlin_empty_speed_, 0.1F);
      getSdfParam(
        sdf, "perlinEmptyThreshold", perlin_empty_threshold_, 0.8F);
      getSdfParam(sdf, "blurSize", blur_size_, 15U);
      getSdfParam(sdf, "erosionSize", erosion_size_, 5U);

      const unsigned int seed =
        std::chrono::system_clock::now().time_since_epoch().count();
      random_generator_ = std::default_random_engine(seed);
      normal_distribution_ =
        std::normal_distribution<float>(0.0F, noise_per_meter_ / 3.0F);
      perlin_noise_ = PerlinNoise(seed);
    } else {
      scaling_ = 1U;
    }
  }

  void AllocateDepthBuffers()
  {
    const unsigned int image_width = depth_camera_->ImageWidth();
    const unsigned int image_height = depth_camera_->ImageHeight();

    depth_map_.resize(
      scaling_ * image_width * scaling_ * image_height);

    if (use_realistic_) {
      small_depth_map_.resize(image_width * image_height);
    }
  }

  void ConfigureGazeboTransport()
  {
    transport_node_.reset(new transport::Node());
    transport_node_->Init(world_->Name());

    const std::string topic_root =
      "~/" + model_->GetName() + "/rs/stream/";

    // The additional separator is retained to preserve the original Gazebo
    // transport topic strings.
    depth_publisher_ = transport_node_->Advertise<msgs::ImageStamped>(
      topic_root + "/" + kDepthCameraTopic,
      1,
      kDepthPublishFrequencyHz);
    infrared1_publisher_ = transport_node_->Advertise<msgs::ImageStamped>(
      topic_root + "/" + kInfrared1CameraTopic,
      1,
      kInfrared1PublishFrequencyHz);
    infrared2_publisher_ = transport_node_->Advertise<msgs::ImageStamped>(
      topic_root + "/" + kInfrared2CameraTopic,
      1,
      kInfrared2PublishFrequencyHz);
    color_publisher_ = transport_node_->Advertise<msgs::ImageStamped>(
      topic_root + "/" + kColorCameraTopic,
      1,
      kColorPublishFrequencyHz);
  }

  void ConnectSensorCallbacks()
  {
    if (use_realistic_) {
      new_depth_frame_connection_ = depth_camera_->ConnectNewDepthFrame(
        std::bind(
          &RealSensePlugin::OnNewDepthFrameRealistic,
          this,
          depth_camera_,
          depth_publisher_));
    } else {
      new_depth_frame_connection_ = depth_camera_->ConnectNewDepthFrame(
        std::bind(
          &RealSensePlugin::OnNewDepthFrame,
          this,
          depth_camera_,
          depth_publisher_));
    }

    for (
      unsigned int index = 0;
      index < infrared_stereo_camera_->CameraCount();
      ++index)
    {
      infrared_cameras_.push_back(infrared_stereo_camera_->Camera(index));
      const std::string camera_name = infrared_cameras_[index]->Name();

      if (camera_name.find(infrared1_camera_plugin_name_) != std::string::npos) {
        new_infrared1_frame_connection_ =
          infrared_cameras_[index]->ConnectNewImageFrame(
          std::bind(
            &RealSensePlugin::OnNewFrame,
            this,
            infrared_cameras_[index],
            infrared1_publisher_));
      } else if (
        camera_name.find(infrared2_camera_plugin_name_) != std::string::npos)
      {
        new_infrared2_frame_connection_ =
          infrared_cameras_[index]->ConnectNewImageFrame(
          std::bind(
            &RealSensePlugin::OnNewFrame,
            this,
            infrared_cameras_[index],
            infrared2_publisher_));
      }
    }

    infrared_stereo_camera_->SetActive(true);
    new_color_frame_connection_ = color_camera_->ConnectNewImageFrame(
      std::bind(
        &RealSensePlugin::OnNewFrame,
        this,
        color_camera_,
        color_publisher_));
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RealSensePlugin::OnUpdate, this));
  }

  bool IsDepthInvalid(float depth) const
  {
    return
      depth <= depth_camera_->NearClip() ||
      depth >= depth_camera_->FarClip() ||
      depth > kDepthScaleMeters * std::numeric_limits<uint16_t>::max() ||
      depth < 0.0F;
  }

  void ApplyDepthBlur(cv::Mat & depth_image) const
  {
    if (blur_size_ == 0U) {
      return;
    }

    // Normalize the blurred depth by the blurred validity mask so that empty
    // pixels do not pull valid measurements toward zero.
    cv::Mat valid_mask;
    cv::Mat blurred_depth;
    cv::Mat blurred_mask;

    cv::compare(depth_image, 0, valid_mask, cv::CMP_NE);
    cv::GaussianBlur(
      depth_image,
      blurred_depth,
      cv::Size(blur_size_, blur_size_),
      2 * blur_size_);
    cv::GaussianBlur(
      valid_mask,
      blurred_mask,
      cv::Size(blur_size_, blur_size_),
      2 * blur_size_);
    cv::divide(
      blurred_depth,
      blurred_mask,
      blurred_depth,
      255.0,
      CV_16UC1);
    blurred_depth.copyTo(depth_image, valid_mask);
  }

  void ApplyDepthErosion(cv::Mat & depth_image) const
  {
    if (erosion_size_ == 0U) {
      return;
    }

    const cv::Mat element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(erosion_size_, erosion_size_));
    cv::erode(depth_image, depth_image, element);
  }

  void PublishGazeboDepthImage(
    const transport::PublisherPtr & publisher,
    const uint16_t * depth_data,
    unsigned int width,
    unsigned int height,
    unsigned int image_size)
  {
    msgs::ImageStamped message;
    msgs::Set(message.mutable_time(), world_->SimTime());

    auto * image = message.mutable_image();
    image->set_width(width);
    image->set_height(height);
    image->set_pixel_format(common::Image::L_INT16);

    // Keep the original step calculation for Gazebo transport compatibility.
    image->set_step(width * height * 2U);
    image->set_data(depth_data, sizeof(uint16_t) * image_size);

    publisher->Publish(message);
  }
};

class GazeboRosRealsense : public RealSensePlugin
{
public:
  GazeboRosRealsense() = default;

  ~GazeboRosRealsense() override
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Unloaded");
  }

  /// Configure ROS publishers, camera frames, and static transforms.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    ros_node_ = gazebo_ros::Node::Get(sdf);
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Realsense Gazebo ROS 2 plugin loading...");

    RealSensePlugin::Load(model, sdf);

    camera_name_ =
      sdf->HasElement("camera_name") ?
      sdf->Get<std::string>("camera_name") : "rgbd";
    camera_suffix_ =
      sdf->HasElement("camera_suffix") ?
      sdf->Get<std::string>("camera_suffix") : "";
    const std::string ros_namespace =
      sdf->HasElement("namespace") ?
      sdf->Get<std::string>("namespace") : "";

    ConfigureFrameIds(ros_namespace);
    ConfigureParentTransform(sdf);
    ConfigureRosPublishers(ros_namespace);

    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(ros_node_);
    CreateStaticTransforms();
  }

  void OnNewFrame(
    const rendering::CameraPtr camera,
    const transport::PublisherPtr publisher) override
  {
    (void)publisher;

    const rclcpp::Time current_time = ros_node_->now();
    std::string camera_frame_id = camera->Name();
    image_transport::CameraPublisher * image_publisher = nullptr;

    if (
      camera_frame_id.find(color_camera_plugin_name_) != std::string::npos)
    {
      camera_frame_id = color_camera_optical_frame_id_;
      image_publisher = &color_publisher_ros_;
    } else if (
      camera_frame_id.find(infrared1_camera_plugin_name_) != std::string::npos)
    {
      camera_frame_id = infrared1_camera_optical_frame_id_;
      image_publisher = &infrared1_publisher_ros_;
    } else if (
      camera_frame_id.find(infrared2_camera_plugin_name_) != std::string::npos)
    {
      camera_frame_id = infrared2_camera_optical_frame_id_;
      image_publisher = &infrared2_publisher_ros_;
    } else {
      camera_frame_id = depth_camera_optical_frame_id_;
      image_publisher = &depth_publisher_ros_;
    }

    sensor_msgs::msg::Image image;
    image.header.frame_id = camera_frame_id;
    image.header.stamp = current_time;

    std::string pixel_format = camera->ImageFormat();
    if (pixel_format == "L_INT8") {
      pixel_format = sensor_msgs::image_encodings::MONO8;
    } else if (pixel_format == "RGB_INT8") {
      pixel_format = sensor_msgs::image_encodings::RGB8;
    } else {
      pixel_format = sensor_msgs::image_encodings::BGR8;
    }

    sensor_msgs::fillImage(
      image,
      pixel_format,
      camera->ImageHeight(),
      camera->ImageWidth(),
      camera->ImageDepth() * camera->ImageWidth(),
      reinterpret_cast<const void *>(camera->ImageData()));

    const sensor_msgs::msg::CameraInfo camera_info =
      CreateCameraInfo(image, camera);
    image_publisher->publish(image, camera_info);
  }

  void OnNewDepthFrame(
    const rendering::CameraPtr camera,
    const transport::PublisherPtr publisher) override
  {
    const rclcpp::Time current_time = ros_node_->now();
    RealSensePlugin::OnNewDepthFrame(camera, publisher);
    PublishRosDepthData(current_time, camera);
  }

  void OnNewDepthFrameRealistic(
    const rendering::CameraPtr camera,
    const transport::PublisherPtr publisher) override
  {
    const rclcpp::Time current_time = ros_node_->now();
    RealSensePlugin::OnNewDepthFrameRealistic(camera, publisher);
    PublishRosDepthData(current_time, camera);
  }

private:
  void ConfigureFrameIds(const std::string & ros_namespace)
  {
    const std::string frame_prefix =
      ros_namespace + "/" + camera_name_ + camera_suffix_ + "/";

    depth_camera_frame_id_ = frame_prefix + kDepthCameraSuffix;
    color_camera_frame_id_ = frame_prefix + kColorCameraSuffix;
    infrared1_camera_frame_id_ = frame_prefix + kInfrared1CameraSuffix;
    infrared2_camera_frame_id_ = frame_prefix + kInfrared2CameraSuffix;
    base_frame_id_ = frame_prefix + kBaseFrameSuffix;

    depth_camera_optical_frame_id_ =
      depth_camera_frame_id_ + "_optical";
    color_camera_optical_frame_id_ =
      color_camera_frame_id_ + "_optical";
    infrared1_camera_optical_frame_id_ =
      infrared1_camera_frame_id_ + "_optical";
    infrared2_camera_optical_frame_id_ =
      infrared2_camera_frame_id_ + "_optical";
  }

  void ConfigureParentTransform(sdf::ElementPtr sdf)
  {
    parent_frame_name_ =
      sdf->HasElement("parentFrameName") ?
      sdf->Get<std::string>("parentFrameName") : "world";

    x_ = sdf->HasElement("x") ? sdf->Get<double>("x") : 0.0;
    y_ = sdf->HasElement("y") ? sdf->Get<double>("y") : 0.0;
    z_ = sdf->HasElement("z") ? sdf->Get<double>("z") : 0.0;
    roll_ =
      sdf->HasElement("roll") ? sdf->Get<double>("roll") : 0.0;
    pitch_ =
      sdf->HasElement("pitch") ? sdf->Get<double>("pitch") : 0.0;
    yaw_ =
      sdf->HasElement("yaw") ? sdf->Get<double>("yaw") : 0.0;
  }

  void ConfigureRosPublishers(const std::string & ros_namespace)
  {
    camera_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(
      ros_node_.get(),
      ros_namespace + "/" + camera_name_ + camera_suffix_);

    image_transport_ =
      std::make_unique<image_transport::ImageTransport>(ros_node_);

    color_publisher_ros_ =
      image_transport_->advertiseCamera(
      camera_name_ + "/color/image_raw", 2);
    infrared1_publisher_ros_ =
      image_transport_->advertiseCamera(
      camera_name_ + "/infra1/image_raw", 2);
    infrared2_publisher_ros_ =
      image_transport_->advertiseCamera(
      camera_name_ + "/infra2/image_raw", 2);
    depth_publisher_ros_ =
      image_transport_->advertiseCamera(
      camera_name_ + "/aligned_depth_to_color/image_raw", 2);

    // SensorDataQoS keeps the point cloud publisher suitable for high-rate
    // sensor traffic and preserves the original best-effort behavior.
    point_cloud_publisher_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      camera_name_ + "/lidar",
      rclcpp::SensorDataQoS());
  }

  void CreateStaticTransforms()
  {
    const rclcpp::Time timestamp = ros_node_->now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    transforms.push_back(
      MakeTransform(
        timestamp,
        parent_frame_name_,
        base_frame_id_,
        x_,
        y_,
        z_,
        roll_,
        pitch_,
        yaw_));
    transforms.push_back(
      MakeTransform(
        timestamp,
        base_frame_id_,
        depth_camera_frame_id_,
        0.0,
        -0.0115,
        0.0,
        0.0,
        0.0,
        0.0));
    transforms.push_back(
      MakeTransform(
        timestamp,
        base_frame_id_,
        color_camera_frame_id_,
        0.0,
        -0.0115,
        0.0,
        0.0,
        0.0,
        0.0));
    transforms.push_back(
      MakeTransform(
        timestamp,
        base_frame_id_,
        infrared1_camera_frame_id_,
        0.0,
        0.0175,
        0.0,
        0.0,
        0.0,
        0.0));
    transforms.push_back(
      MakeTransform(
        timestamp,
        base_frame_id_,
        infrared2_camera_frame_id_,
        0.0,
        -0.0325,
        0.0,
        0.0,
        0.0,
        0.0));

    // ROS optical frames use x-right, y-down, and z-forward axes.
    transforms.push_back(
      MakeTransform(
        timestamp,
        depth_camera_frame_id_,
        depth_camera_optical_frame_id_,
        0.0,
        0.0,
        0.0,
        -kHalfPi,
        0.0,
        -kHalfPi));
    transforms.push_back(
      MakeTransform(
        timestamp,
        color_camera_frame_id_,
        color_camera_optical_frame_id_,
        0.0,
        0.0,
        0.0,
        -kHalfPi,
        0.0,
        -kHalfPi));
    transforms.push_back(
      MakeTransform(
        timestamp,
        infrared1_camera_frame_id_,
        infrared1_camera_optical_frame_id_,
        0.0,
        0.0,
        0.0,
        -kHalfPi,
        0.0,
        -kHalfPi));
    transforms.push_back(
      MakeTransform(
        timestamp,
        infrared2_camera_frame_id_,
        infrared2_camera_optical_frame_id_,
        0.0,
        0.0,
        0.0,
        -kHalfPi,
        0.0,
        -kHalfPi));

    static_tf_broadcaster_->sendTransform(transforms);
  }

  static geometry_msgs::msg::TransformStamped MakeTransform(
    const rclcpp::Time & timestamp,
    const std::string & parent_frame,
    const std::string & child_frame,
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = timestamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    return transform;
  }

  sensor_msgs::msg::CameraInfo CreateCameraInfo(
    const sensor_msgs::msg::Image & image,
    const rendering::CameraPtr camera) const
  {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = image.header;
    camera_info.height = image.height;
    camera_info.width = image.width;

    const double horizontal_fov = camera->HFOV().Radian();
    const double horizontal_focal_length =
      (image.width / 2.0) / std::tan(horizontal_fov / 2.0);
    const double vertical_fov = camera->VFOV().Radian();
    const double vertical_focal_length =
      (image.height / 2.0) / std::tan(vertical_fov / 2.0);

    camera_info.distortion_model =
      sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

    camera_info.k.fill(0.0);
    camera_info.k[0] = horizontal_focal_length;
    camera_info.k[4] = vertical_focal_length;
    camera_info.k[2] = (camera_info.width + 1.0) / 2.0;
    camera_info.k[5] = (camera_info.height + 1.0) / 2.0;
    camera_info.k[8] = 1.0;

    camera_info.p.fill(0.0);
    camera_info.p[0] = camera_info.k[0];
    camera_info.p[5] = camera_info.k[4];
    camera_info.p[2] = camera_info.k[2];
    camera_info.p[6] = camera_info.k[5];
    camera_info.p[10] = camera_info.k[8];

    camera_info.d.assign(5, 0.0);
    return camera_info;
  }

  void PublishRosDepthData(
    const rclcpp::Time & current_time,
    const rendering::CameraPtr camera)
  {
    sensor_msgs::msg::Image depth_image;
    depth_image.header.frame_id = depth_camera_optical_frame_id_;
    depth_image.header.stamp = current_time;

    sensor_msgs::fillImage(
      depth_image,
      sensor_msgs::image_encodings::TYPE_16UC1,
      scaling_ * depth_camera_->ImageHeight(),
      scaling_ * depth_camera_->ImageWidth(),
      sizeof(uint16_t) * scaling_ * depth_camera_->ImageWidth(),
      reinterpret_cast<const void *>(depth_map_.data()));

    const sensor_msgs::msg::CameraInfo camera_info =
      CreateCameraInfo(depth_image, camera);
    depth_publisher_ros_.publish(depth_image, camera_info);

    // The point cloud is generated from the same depth buffer, so the
    // realistic path automatically includes its configured artifacts.
    PublishPointCloud(
      current_time,
      camera_info,
      depth_map_.data(),
      depth_image.width,
      depth_image.height);
  }

  void PublishPointCloud(
    const rclcpp::Time & current_time,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const uint16_t * depth_data,
    unsigned int width,
    unsigned int height)
  {
    const unsigned int cloud_width = width / kPointCloudDecimation;
    const unsigned int cloud_height = height / kPointCloudDecimation;
    const unsigned int point_count = cloud_width * cloud_height;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = current_time;
    cloud.header.frame_id = depth_camera_optical_frame_id_;
    cloud.height = 1U;
    cloud.width = point_count;
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");

    const double focal_x = camera_info.k[0];
    const double focal_y = camera_info.k[4];
    const double center_x = camera_info.k[2];
    const double center_y = camera_info.k[5];
    const float invalid_value = std::numeric_limits<float>::quiet_NaN();

    // Decimation samples one pixel from each block without changing the
    // original projection model or the unorganized cloud layout.
    for (unsigned int cloud_v = 0; cloud_v < cloud_height; ++cloud_v) {
      const unsigned int image_v = cloud_v * kPointCloudDecimation;

      for (unsigned int cloud_u = 0; cloud_u < cloud_width; ++cloud_u) {
        const unsigned int image_u = cloud_u * kPointCloudDecimation;
        const uint16_t depth = depth_data[image_v * width + image_u];

        if (depth == 0U) {
          *x_iterator = invalid_value;
          *y_iterator = invalid_value;
          *z_iterator = invalid_value;
        } else {
          const float z = depth * kDepthScaleMeters;
          *x_iterator = (image_u - center_x) * z / focal_x;
          *y_iterator = (image_v - center_y) * z / focal_y;
          *z_iterator = z;
        }

        ++x_iterator;
        ++y_iterator;
        ++z_iterator;
      }
    }

    point_cloud_publisher_->publish(cloud);
  }

  gazebo_ros::Node::SharedPtr ros_node_{nullptr};
  std::shared_ptr<camera_info_manager::CameraInfoManager>
  camera_info_manager_{nullptr};
  std::unique_ptr<image_transport::ImageTransport> image_transport_{nullptr};

  image_transport::CameraPublisher color_publisher_ros_;
  image_transport::CameraPublisher infrared1_publisher_ros_;
  image_transport::CameraPublisher infrared2_publisher_ros_;
  image_transport::CameraPublisher depth_publisher_ros_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
  point_cloud_publisher_{nullptr};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
  static_tf_broadcaster_{nullptr};

  std::string camera_name_{"rgbd"};
  std::string camera_suffix_;
  std::string parent_frame_name_;

  std::string depth_camera_frame_id_;
  std::string color_camera_frame_id_;
  std::string infrared1_camera_frame_id_;
  std::string infrared2_camera_frame_id_;
  std::string base_frame_id_;

  std::string depth_camera_optical_frame_id_;
  std::string color_camera_optical_frame_id_;
  std::string infrared1_camera_optical_frame_id_;
  std::string infrared2_camera_optical_frame_id_;

  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

}  // namespace gazebo
