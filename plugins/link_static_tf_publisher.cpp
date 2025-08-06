#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo
{

class LinkStaticTFPublisher : public ModelPlugin
{
private:
  physics::ModelPtr model;
  std::shared_ptr<rclcpp::Node> ros_node;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;

public:
  LinkStaticTFPublisher() {}

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Safety check
    if (!_model)
    {
      std::cerr << "[LinkStaticTFPublisher]: Invalid model pointer!" << std::endl;
      return;
    }

    model = _model;

    std::string parentLinkName;
    std::string childLinkName;
    std::string robotNamespace = "/";

    if (_sdf->HasElement("robotNamespace"))
    {
      robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    if (_sdf->HasElement("parentLink"))
    {
      parentLinkName = _sdf->Get<std::string>("parentLink");
    }
    else
    {
      std::cerr << "[LinkStaticTFPublisher]: SDF is missing element \"parentLink\"" << std::endl;
      return;
    }

    if (_sdf->HasElement("childLink"))
    {
      childLinkName = _sdf->Get<std::string>("childLink");
    }
    else
    {
      std::cerr << "[LinkStaticTFPublisher]: SDF is missing element \"childLink\"" << std::endl;
      return;
    }

    // Get pointers to the specified parent and child links
    physics::LinkPtr parentLink = model->GetLink(parentLinkName);
    physics::LinkPtr childLink = model->GetLink(childLinkName);

    if (!parentLink || !childLink)
    {
      std::cerr << "[LinkStaticTFPublisher]: One or both of the links \""
                << parentLinkName << "\", \"" << childLinkName << "\" do not exist!" << std::endl;
      return;
    }

    // Initialize ROS 2 node
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    ros_node = std::make_shared<rclcpp::Node>("link_static_tf_publisher");
    tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(ros_node);

    // Get the transform between the parent and child links
    ignition::math::Pose3d relativePose = childLink->WorldPose() - parentLink->WorldPose();

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = ros_node->get_clock()->now();
    transformStamped.header.frame_id = robotNamespace + "/" + parentLinkName.erase(parentLinkName.find("_link"), std::string("_link").length());
    transformStamped.child_frame_id = robotNamespace + "/" + childLinkName.erase(childLinkName.find("_link"), std::string("_link").length());
    transformStamped.transform.translation.x = relativePose.Pos().X();
    transformStamped.transform.translation.y = relativePose.Pos().Y();
    transformStamped.transform.translation.z = relativePose.Pos().Z();
    transformStamped.transform.rotation.w = relativePose.Rot().W();
    transformStamped.transform.rotation.x = relativePose.Rot().X();
    transformStamped.transform.rotation.y = relativePose.Rot().Y();
    transformStamped.transform.rotation.z = relativePose.Rot().Z();

    tf_broadcaster->sendTransform(transformStamped);

    RCLCPP_INFO(ros_node->get_logger(),
                "[LinkStaticTFPublisher]: Published static TF between \"%s\" and \"%s\"",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());
  }
};

GZ_REGISTER_MODEL_PLUGIN(LinkStaticTFPublisher)
} // namespace gazebo
