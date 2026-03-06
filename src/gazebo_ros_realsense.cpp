#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <string>
#include <memory>
#include <random>
#include <chrono>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>

#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <gazebo_ros/node.hpp>

#include "laser_gazebo_resources/perlin_noise.h"
#include "laser_gazebo_resources/common.h"

#define DEPTH_CAMERA_TOPIC "aligned_depth_to_color"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infra1"
#define IRED2_CAMERA_TOPIC "infra2"
#define DEPTH_PUB_FREQ_HZ 30
#define COLOR_PUB_FREQ_HZ 30
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define DEPTH_SCALE_M 0.001

namespace gazebo
{

const std::string DEPTH_CAMERA_SUFFIX = "aligned_depth_to_color";
const std::string COLOR_CAMERA_SUFFIX = "color";
const std::string IREDS_CAMERA_SUFFIX = "infra_stereo";
const std::string IRED1_CAMERA_SUFFIX = "infra1";
const std::string IRED2_CAMERA_SUFFIX = "infra2";
const std::string BASE_FRAME_SUFFIX   = "link";

class RealSensePlugin : public ModelPlugin {

  void dm_upscale(const cv::Mat& in, cv::Mat& out, int scale) {
    const unsigned  i_rows     = in.rows;
    const unsigned  i_cols     = in.cols;
    const unsigned  o_rows     = i_rows * scale;
    const unsigned  o_cols     = i_cols * scale;
    cv::Mat         ret        = cv::Mat(int(o_rows), int(o_cols), CV_16UC1);
    const uint16_t* i_data_ptr = in.ptr<uint16_t>(0);

    uint16_t* o_px_ptr = ret.ptr<uint16_t>(0);
    for (unsigned rit = 0; rit < o_rows; rit++) {
      for (unsigned cit = 0; cit < o_cols; cit++) {
        *o_px_ptr = i_data_ptr[cit / scale + (rit / scale) * i_cols];
        o_px_ptr++;
      }
    }
    out = ret;
  }

public:
  RealSensePlugin() {
    this->depthCam      = nullptr;
    this->iredStereoCam = nullptr;
    this->colorCam      = nullptr;
  }

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->rsModel                    = _model;
    this->world                      = this->rsModel->GetWorld();
    sensors::SensorManager* smanager = sensors::SensorManager::Instance();

    std::string camera_name = "rgbd";
    if (_sdf->HasElement("camera_name")) {
      camera_name = _sdf->Get<std::string>("camera_name");
    }

    std::string camera_suffix = "";
    if (_sdf->HasElement("camera_suffix")) {
      camera_suffix = _sdf->Get<std::string>("camera_suffix");
    }

    std::string _namespace = "";
    if (_sdf->HasElement("namespace")) {
      _namespace = _sdf->Get<std::string>("namespace");
    }

    depth_camera_plugin_name_ = _namespace + "/" + camera_name + camera_suffix + "_" + DEPTH_CAMERA_SUFFIX;
    color_camera_plugin_name_ = _namespace + "/" + camera_name + camera_suffix + "_" + COLOR_CAMERA_SUFFIX;
    ireds_camera_plugin_name_ = _namespace + "/" + camera_name + camera_suffix + "_" + IREDS_CAMERA_SUFFIX;
    ired1_camera_plugin_name_ = _namespace + "/" + camera_name + camera_suffix + "_" + IRED1_CAMERA_SUFFIX;
    ired2_camera_plugin_name_ = _namespace + "/" + camera_name + camera_suffix + "_" + IRED2_CAMERA_SUFFIX;

    const auto depthPtr = smanager->GetSensor(depth_camera_plugin_name_);
    const auto colorPtr = smanager->GetSensor(color_camera_plugin_name_);
    const auto iredsPtr = smanager->GetSensor(ireds_camera_plugin_name_);

    if (!depthPtr || !colorPtr || !iredsPtr) {
      gzerr << "RealSensePlugin: One or more sensors not found!" << std::endl;
      return;
    }

    this->depthCam      = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(depthPtr)->DepthCamera();
    this->iredStereoCam = std::dynamic_pointer_cast<sensors::MultiCameraSensor>(iredsPtr);
    this->colorCam      = std::dynamic_pointer_cast<sensors::CameraSensor>(colorPtr)->Camera();

    getSdfParam(_sdf, "useRealistic", this->useRealistic, false);

    if (this->useRealistic) {
      getSdfParam(_sdf, "imageScaling", this->scaling, 4u);
      getSdfParam(_sdf, "noisePerMeter", this->noisePerMeter, 0.2f);
      getSdfParam(_sdf, "noiseMinDistance", this->noiseMinDistance, 4.0f);
      getSdfParam(_sdf, "perlinEmptySpeed", this->perlinEmptySpeed, 0.1f);
      getSdfParam(_sdf, "perlinEmptyThreshold", this->perlinEmptyThreshold, 0.8f);
      getSdfParam(_sdf, "blurSize", this->blurSize, 15u);
      getSdfParam(_sdf, "erosionSize", this->erosionSize, 5u);

      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      rand_gen      = std::default_random_engine(seed);
      randn_dist    = std::normal_distribution<float>(0.0f, noisePerMeter / 3);
      perlinNoise   = PerlinNoise(seed);
    } else {
      this->scaling = 1;
    }

    this->depthMap.resize(this->scaling * this->depthCam->ImageWidth() * this->scaling * this->depthCam->ImageHeight());
    if (this->useRealistic)
      this->depthMapSmall.resize(this->depthCam->ImageWidth() * this->depthCam->ImageHeight());

    this->transportNode = transport::NodePtr(new transport::Node());
    this->transportNode->Init(this->world->Name());

    std::string rsTopicRoot = "~/" + this->rsModel->GetName() + "/rs/stream/";
    this->depthPub          = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + "/" + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
    this->ired1Pub          = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + "/" + IRED1_CAMERA_TOPIC, 1, IRED1_PUB_FREQ_HZ);
    this->ired2Pub          = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + "/" + IRED2_CAMERA_TOPIC, 1, IRED2_PUB_FREQ_HZ);
    this->colorPub          = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + "/" + COLOR_CAMERA_TOPIC, 1, COLOR_PUB_FREQ_HZ);

    if (this->useRealistic) {
      this->newDepthFrameConn =
          this->depthCam->ConnectNewDepthFrame(std::bind(&RealSensePlugin::OnNewDepthFrameRealistic, this, this->depthCam, this->depthPub));
    } else {
      this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(std::bind(&RealSensePlugin::OnNewDepthFrame, this, this->depthCam, this->depthPub));
    }

    for (unsigned int i = 0; i < this->iredStereoCam->CameraCount(); ++i) {
      this->iredCams.push_back(this->iredStereoCam->Camera(i));
      std::string cameraName = this->iredStereoCam->Camera(i)->Name();
      if (cameraName.find(ired1_camera_plugin_name_) != std::string::npos) {
        this->newIred1FrameConn = this->iredCams[i]->ConnectNewImageFrame(std::bind(&RealSensePlugin::OnNewFrame, this, this->iredCams[i], this->ired1Pub));
      } else if (cameraName.find(ired2_camera_plugin_name_) != std::string::npos) {
        this->newIred2FrameConn = this->iredCams[i]->ConnectNewImageFrame(std::bind(&RealSensePlugin::OnNewFrame, this, this->iredCams[i], this->ired2Pub));
      }
    }

    this->iredStereoCam->SetActive(true);
    this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(&RealSensePlugin::OnNewFrame, this, this->colorCam, this->colorPub));
    this->updateConnection  = event::Events::ConnectWorldUpdateBegin(std::bind(&RealSensePlugin::OnUpdate, this));
  }

  virtual void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), this->world->SimTime());
    msg.mutable_image()->set_width(cam->ImageWidth());
    msg.mutable_image()->set_height(cam->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(cam->ImageFormat()));
    msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
    msg.mutable_image()->set_data(cam->ImageData(), cam->ImageDepth() * cam->ImageWidth() * cam->ImageHeight());
    pub->Publish(msg);
  }

  virtual void OnNewDepthFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    const unsigned     imageWidth  = this->depthCam->ImageWidth();
    const unsigned     imageHeight = this->depthCam->ImageHeight();
    const unsigned     imageSize   = imageWidth * imageHeight;
    msgs::ImageStamped msg;
    const float*       depthDataFloat = this->depthCam->DepthData();
    for (unsigned int i = 0; i < imageSize; ++i) {
      const float cur_depth = depthDataFloat[i];
      if (cur_depth <= this->depthCam->NearClip() || cur_depth >= this->depthCam->FarClip() || cur_depth > DEPTH_SCALE_M * UINT16_MAX || cur_depth < 0) {
        this->depthMap[i] = 0;
      } else {
        this->depthMap[i] = (cur_depth) / DEPTH_SCALE_M;
      }
    }
    msgs::Set(msg.mutable_time(), this->world->SimTime());
    msg.mutable_image()->set_width(imageWidth);
    msg.mutable_image()->set_height(imageHeight);
    msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
    msg.mutable_image()->set_step(imageWidth * imageHeight * 2);
    msg.mutable_image()->set_data(this->depthMap.data(), sizeof(uint16_t) * imageSize);
    pub->Publish(msg);
  }

  virtual void OnNewDepthFrameRealistic(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    const unsigned imageWidth  = this->depthCam->ImageWidth();
    const unsigned imageHeight = this->depthCam->ImageHeight();
    const unsigned imageSize   = imageWidth * imageHeight;
    static float   cur_z       = 6.0;
    cur_z += this->perlinEmptySpeed;
    msgs::ImageStamped msg;
    const float*       depthDataFloat = this->depthCam->DepthData();
    unsigned           x = 0, y = 0;
    for (unsigned int i = 0; i < imageSize; ++i) {
      const float x_rel     = 5.0f * float(x) / float(imageWidth);
      const float y_rel     = 5.0f * float(y) / float(imageHeight);
      const float cur_depth = depthDataFloat[i];
      if (cur_depth <= this->depthCam->NearClip() || cur_depth >= this->depthCam->FarClip() || cur_depth > DEPTH_SCALE_M * UINT16_MAX || cur_depth < 0 ||
          this->perlinNoise.noise(x_rel, y_rel, cur_z) > this->perlinEmptyThreshold) {
        this->depthMapSmall[i] = 0;
      } else {
        const float noise_scale = cur_depth > this->noiseMinDistance ? cur_depth : this->noiseMinDistance;
        const float noise       = noise_scale * this->randn_dist(rand_gen);
        float       noisy_depth = (noise + cur_depth);
        if (noisy_depth < 0 || noisy_depth > DEPTH_SCALE_M * UINT16_MAX)
          noisy_depth = 0;
        this->depthMapSmall[i] = noisy_depth / DEPTH_SCALE_M;
      }
      x++;
      if (x >= imageWidth) {
        x = 0;
        y++;
      }
    }
    const unsigned n_width  = depthCam->ImageWidth() * this->scaling;
    const unsigned n_height = depthCam->ImageHeight() * this->scaling;
    const unsigned n_imsize = n_width * n_height;
    cv::Mat        im(int(depthCam->ImageHeight()), int(depthCam->ImageWidth()), CV_16UC1, this->depthMapSmall.data());
    dm_upscale(im, im, int(this->scaling));
    if (this->blurSize != 0) {
      cv::Mat mask, blr_im, blr_mask;
      cv::compare(im, 0, mask, cv::CMP_NE);
      cv::GaussianBlur(im, blr_im, cv::Size(this->blurSize, this->blurSize), 2 * this->blurSize);
      cv::GaussianBlur(mask, blr_mask, cv::Size(this->blurSize, this->blurSize), 2 * this->blurSize);
      cv::divide(blr_im, blr_mask, blr_im, 255.0, CV_16UC1);
      blr_im.copyTo(im, mask);
    }
    if (this->erosionSize != 0) {
      cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(this->erosionSize, this->erosionSize));
      cv::erode(im, im, element);
    }
    memcpy(this->depthMap.data(), im.ptr(0), sizeof(uint16_t) * n_imsize);
    msgs::Set(msg.mutable_time(), this->world->SimTime());
    msg.mutable_image()->set_width(n_width);
    msg.mutable_image()->set_height(n_height);
    msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
    msg.mutable_image()->set_step(n_width * n_height * 2);
    msg.mutable_image()->set_data(this->depthMap.data(), sizeof(uint16_t) * n_imsize);
    pub->Publish(msg);
  }

  void OnUpdate() {
  }

protected:
  std::string depth_camera_plugin_name_, color_camera_plugin_name_, ireds_camera_plugin_name_, ired1_camera_plugin_name_, ired2_camera_plugin_name_;
  bool        useRealistic;
  unsigned    scaling = 4;
  float       noisePerMeter, noiseMinDistance, perlinEmptySpeed, perlinEmptyThreshold;
  unsigned    blurSize, erosionSize;
  PerlinNoise perlinNoise;
  std::default_random_engine        rand_gen;
  std::normal_distribution<float>   randn_dist;
  physics::ModelPtr                 rsModel;
  physics::WorldPtr                 world;
  rendering::DepthCameraPtr         depthCam;
  rendering::CameraPtr              colorCam;
  sensors::MultiCameraSensorPtr     iredStereoCam;
  std::vector<rendering::CameraPtr> iredCams;
  transport::NodePtr                transportNode;
  std::vector<uint16_t>             depthMap, depthMapSmall;
  transport::PublisherPtr           depthPub, colorPub, ired1Pub, ired2Pub;
  event::ConnectionPtr              newDepthFrameConn, newIred1FrameConn, newIred2FrameConn, newColorFrameConn, updateConnection;
};

class GazeboRosRealsense : public RealSensePlugin {
public:
  GazeboRosRealsense() {
  }
  ~GazeboRosRealsense() {
    RCLCPP_DEBUG(this->ros_node_->get_logger(), "Unloaded");
  }

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(this->ros_node_->get_logger(), "Realsense Gazebo ROS 2 plugin loading...");
    RealSensePlugin::Load(_model, _sdf);

    if (!_sdf->HasElement("camera_name")) {
      camera_name_ = "rgbd";
    } else {
      camera_name_ = _sdf->Get<std::string>("camera_name");
    }

    if (!_sdf->HasElement("camera_suffix")) {
      camera_suffix_ = "";
    } else {
      camera_suffix_ = _sdf->Get<std::string>("camera_suffix");
    }

    std::string _namespace = "";
    if (_sdf->HasElement("namespace")) {
      _namespace = _sdf->Get<std::string>("namespace");
    }

    depth_camera_frame_id_ = _namespace + "/" + camera_name_ + camera_suffix_ + "/" + DEPTH_CAMERA_SUFFIX;
    color_camera_frame_id_ = _namespace + "/" + camera_name_ + camera_suffix_ + "/" + COLOR_CAMERA_SUFFIX;
    ired1_camera_frame_id_ = _namespace + "/" + camera_name_ + camera_suffix_ + "/" + IRED1_CAMERA_SUFFIX;
    ired2_camera_frame_id_ = _namespace + "/" + camera_name_ + camera_suffix_ + "/" + IRED2_CAMERA_SUFFIX;
    base_frame_id_         = _namespace + "/" + camera_name_ + camera_suffix_ + "/" + BASE_FRAME_SUFFIX;

    depth_camera_optical_frame_id_ = depth_camera_frame_id_ + "_optical";
    color_camera_optical_frame_id_ = color_camera_frame_id_ + "_optical";
    ired1_camera_optical_frame_id_ = ired1_camera_frame_id_ + "_optical";
    ired2_camera_optical_frame_id_ = ired2_camera_frame_id_ + "_optical";

    if (!_sdf->HasElement("parentFrameName"))
      this->parent_frame_name_ = "world";
    else
      this->parent_frame_name_ = _sdf->Get<std::string>("parentFrameName");

    this->x_     = _sdf->HasElement("x") ? _sdf->Get<double>("x") : 0.0;
    this->y_     = _sdf->HasElement("y") ? _sdf->Get<double>("y") : 0.0;
    this->z_     = _sdf->HasElement("z") ? _sdf->Get<double>("z") : 0.0;
    this->roll_  = _sdf->HasElement("roll") ? _sdf->Get<double>("roll") : 0.0;
    this->pitch_ = _sdf->HasElement("pitch") ? _sdf->Get<double>("pitch") : 0.0;
    this->yaw_   = _sdf->HasElement("yaw") ? _sdf->Get<double>("yaw") : 0.0;

    this->camera_info_manager_ =
        std::make_shared<camera_info_manager::CameraInfoManager>(this->ros_node_.get(), _namespace + "/" + camera_name_ + camera_suffix_);
    this->itnode_ = std::make_unique<image_transport::ImageTransport>(this->ros_node_);

    this->color_pub_ = this->itnode_->advertiseCamera(camera_name_ + "/color/image_raw", 2);
    this->ir1_pub_   = this->itnode_->advertiseCamera(camera_name_ + "/infra1/image_raw", 2);
    this->ir2_pub_   = this->itnode_->advertiseCamera(camera_name_ + "/infra2/image_raw", 2);
    this->depth_pub_ = this->itnode_->advertiseCamera(camera_name_ + "/aligned_depth_to_color/image_raw", 2);

    this->static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->ros_node_);
    createStaticTransforms();
  }

  void createStaticTransforms() {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    rclcpp::Time                                      now = this->ros_node_->now();

    auto make_tf = [&](const std::string& parent, const std::string& child, double tx, double ty, double tz, double r, double p, double y_rot) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp            = now;
      ts.header.frame_id         = parent;
      ts.child_frame_id          = child;
      ts.transform.translation.x = tx;
      ts.transform.translation.y = ty;
      ts.transform.translation.z = tz;
      tf2::Quaternion q;
      q.setRPY(r, p, y_rot);
      ts.transform.rotation.x = q.x();
      ts.transform.rotation.y = q.y();
      ts.transform.rotation.z = q.z();
      ts.transform.rotation.w = q.w();
      return ts;
    };

    transforms.push_back(make_tf(parent_frame_name_, base_frame_id_, x_, y_, z_, roll_, pitch_, yaw_));
    transforms.push_back(make_tf(base_frame_id_, depth_camera_frame_id_, 0, -0.0115, 0, 0, 0, 0));
    transforms.push_back(make_tf(base_frame_id_, color_camera_frame_id_, 0, -0.0115, 0, 0, 0, 0));
    transforms.push_back(make_tf(base_frame_id_, ired1_camera_frame_id_, 0, 0.0175, 0, 0, 0, 0));
    transforms.push_back(make_tf(base_frame_id_, ired2_camera_frame_id_, 0, -0.0325, 0, 0, 0, 0));

    transforms.push_back(make_tf(depth_camera_frame_id_, depth_camera_optical_frame_id_, 0, 0, 0, -M_PI_2, 0, -M_PI_2));
    transforms.push_back(make_tf(color_camera_frame_id_, color_camera_optical_frame_id_, 0, 0, 0, -M_PI_2, 0, -M_PI_2));
    transforms.push_back(make_tf(ired1_camera_frame_id_, ired1_camera_optical_frame_id_, 0, 0, 0, -M_PI_2, 0, -M_PI_2));
    transforms.push_back(make_tf(ired2_camera_frame_id_, ired2_camera_optical_frame_id_, 0, 0, 0, -M_PI_2, 0, -M_PI_2));

    this->static_tf_broadcaster_->sendTransform(transforms);
  }

  virtual void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    rclcpp::Time                      current_time = this->ros_node_->now();
    std::string                       camera_id    = cam->Name();
    image_transport::CameraPublisher* image_pub;

    if (camera_id.find(color_camera_plugin_name_) != std::string::npos) {
      camera_id = color_camera_optical_frame_id_;
      image_pub = &(this->color_pub_);
    } else if (camera_id.find(ired1_camera_plugin_name_) != std::string::npos) {
      camera_id = ired1_camera_optical_frame_id_;
      image_pub = &(this->ir1_pub_);
    } else if (camera_id.find(ired2_camera_plugin_name_) != std::string::npos) {
      camera_id = ired2_camera_optical_frame_id_;
      image_pub = &(this->ir2_pub_);
    } else {
      camera_id = depth_camera_optical_frame_id_;
      image_pub = &(this->depth_pub_);
    }

    sensor_msgs::msg::Image img;
    img.header.frame_id = camera_id;
    img.header.stamp    = current_time;

    std::string pixel_format = cam->ImageFormat();
    if (pixel_format == "L_INT8")
      pixel_format = sensor_msgs::image_encodings::MONO8;
    else if (pixel_format == "RGB_INT8")
      pixel_format = sensor_msgs::image_encodings::RGB8;
    else
      pixel_format = sensor_msgs::image_encodings::BGR8;

    sensor_msgs::fillImage(img, pixel_format, cam->ImageHeight(), cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(),
                           reinterpret_cast<const void*>(cam->ImageData()));
    sensor_msgs::msg::CameraInfo cam_info_msg = cameraInfo(img, cam);
    image_pub->publish(img, cam_info_msg);
  }

  virtual void OnNewDepthFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    rclcpp::Time current_time = this->ros_node_->now();
    RealSensePlugin::OnNewDepthFrame(cam, pub);

    sensor_msgs::msg::Image d_msg;
    d_msg.header.frame_id = depth_camera_optical_frame_id_;
    d_msg.header.stamp    = current_time;
    sensor_msgs::fillImage(d_msg, sensor_msgs::image_encodings::TYPE_16UC1, this->scaling * this->depthCam->ImageHeight(),
                           this->scaling * this->depthCam->ImageWidth(), sizeof(uint16_t) * this->scaling * this->depthCam->ImageWidth(),
                           reinterpret_cast<const void*>(this->depthMap.data()));

    sensor_msgs::msg::CameraInfo info = cameraInfo(d_msg, cam);
    this->depth_pub_.publish(d_msg, info);
  }

  virtual void OnNewDepthFrameRealistic(const rendering::CameraPtr cam, const transport::PublisherPtr pub) {
    rclcpp::Time current_time = this->ros_node_->now();
    RealSensePlugin::OnNewDepthFrameRealistic(cam, pub);

    sensor_msgs::msg::Image d_msg;
    d_msg.header.frame_id = depth_camera_optical_frame_id_;
    d_msg.header.stamp    = current_time;
    sensor_msgs::fillImage(d_msg, sensor_msgs::image_encodings::TYPE_16UC1, this->scaling * this->depthCam->ImageHeight(),
                           this->scaling * this->depthCam->ImageWidth(), sizeof(uint16_t) * this->scaling * this->depthCam->ImageWidth(),
                           reinterpret_cast<const void*>(this->depthMap.data()));

    sensor_msgs::msg::CameraInfo info = cameraInfo(d_msg, cam);
    this->depth_pub_.publish(d_msg, info);
  }

  sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image& image, const gazebo::rendering::CameraPtr cam) {
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = image.header;
    info_msg.height = image.height;
    info_msg.width  = image.width;

    double hfov = cam->HFOV().Radian();
    double hfoc = (image.width / 2.0) / tan(hfov / 2.0);
    double vfov = cam->VFOV().Radian();
    double vfoc = (image.height / 2.0) / tan(vfov / 2.0);

    info_msg.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

    // No ROS 2, k é std::array<double, 9>.
    // O método fill(0) limpa o array se necessário, mas não é preciso dar assign.
    info_msg.k.fill(0.0);
    info_msg.k[0] = hfoc;
    info_msg.k[4] = vfoc;
    info_msg.k[2] = (info_msg.width + 1.0) / 2.0;
    info_msg.k[5] = (info_msg.height + 1.0) / 2.0;
    info_msg.k[8] = 1.0;

    // No ROS 2, p é std::array<double, 12>.
    info_msg.p.fill(0.0);
    info_msg.p[0]  = info_msg.k[0];
    info_msg.p[5]  = info_msg.k[4];
    info_msg.p[2]  = info_msg.k[2];
    info_msg.p[6]  = info_msg.k[5];
    info_msg.p[10] = info_msg.k[8];

    // O campo 'd' (distorção) costuma ser um std::vector no ROS 2,
    // então aqui o assign ainda funcionaria, mas geralmente inicia-se vazio.
    info_msg.d.assign(5, 0.0);

    return info_msg;
  }

private:
  gazebo_ros::Node::SharedPtr                             ros_node_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::unique_ptr<image_transport::ImageTransport>        itnode_;
  image_transport::CameraPublisher                        color_pub_, ir1_pub_, ir2_pub_, depth_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>    static_tf_broadcaster_;
  std::string                                             _namespace = "not_linked", camera_name_ = "rgbd", camera_suffix_ = "";
  std::string parent_frame_name_, depth_camera_frame_id_, color_camera_frame_id_, ired1_camera_frame_id_, ired2_camera_frame_id_, base_frame_id_;
  std::string depth_camera_optical_frame_id_, color_camera_optical_frame_id_, ired1_camera_optical_frame_id_, ired2_camera_optical_frame_id_;
  double      x_, y_, z_, roll_, pitch_, yaw_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)
}  // namespace gazebo
