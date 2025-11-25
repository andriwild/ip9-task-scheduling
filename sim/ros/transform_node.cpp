#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <gz/transport.hh>
#include <gz/msgs.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

// ros2 run tf2_tools view_frames

class GazeboTfBroadcaster : public rclcpp::Node {
public:
  GazeboTfBroadcaster()
  : Node("gazebo_tf_broadcaster")
  {
    this->declare_parameter("publish_frequency", 1.0);
    this->declare_parameter("world_name", "imvs");
    this->declare_parameter("robot_name", "dingo");
    this->declare_parameter("base_frame", "base_link");

    _pub_frequency = this->get_parameter("publish_frequency").as_double();
    _world_name = this->get_parameter("world_name").as_string();
    _robot_name = this->get_parameter("robot_name").as_string();
    _base_link = this->get_parameter("base_frame").as_string();

    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to Gazebo topic
    std::string topic = "/world/" + _world_name + "/pose/info";

    if (!_gz_node.Subscribe(topic, &GazeboTfBroadcaster::onPoseInfo, this)) {
      RCLCPP_ERROR(this->get_logger(), "Fehler beim Subscriben auf %s", topic.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Subscribed auf Gazebo topic: %s", topic.c_str());
    }
  }

private:
  void onPoseInfo(const gz::msgs::Pose_V &msg) {
    auto now = this->now();

    // reduce publish frequency
    if (_pub_frequency > 0.0 && _last_published.seconds() > 0.0) {
      double elapsed = (now - _last_published).seconds();
      if (elapsed < 1.0 / _pub_frequency) {
        return;
      }
    }

    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto &pose = msg.pose(i);

      if (pose.name() == _robot_name) {

        // no transform (map -> odom used for drift)
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = now;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.transform.translation.x = 0.0;
        map_to_odom.transform.translation.y = 0.0;
        map_to_odom.transform.translation.z = 0.0;
        map_to_odom.transform.rotation.x = 0.0;
        map_to_odom.transform.rotation.y = 0.0;
        map_to_odom.transform.rotation.z = 0.0;
        map_to_odom.transform.rotation.w = 1.0;

        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = now;
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = _base_link;

        tf2::Quaternion gazebo_quat(
          pose.orientation().x(),
          pose.orientation().y(),
          pose.orientation().z(),
          pose.orientation().w()
        );

        // -90° yaw correction to gazebo
        tf2::Quaternion rotation_correction;
        rotation_correction.setRPY(0, 0, -M_PI / 2);

        tf2::Quaternion corrected_quat = rotation_correction * gazebo_quat;
        corrected_quat.normalize();

        // y and x swapped
        odom_to_base.transform.translation.x = pose.position().y();
        odom_to_base.transform.translation.y = -pose.position().x();
        odom_to_base.transform.translation.z = pose.position().z();

        odom_to_base.transform.rotation.w = corrected_quat.w();
        odom_to_base.transform.rotation.x = corrected_quat.x();
        odom_to_base.transform.rotation.y = corrected_quat.y();
        odom_to_base.transform.rotation.z = corrected_quat.z();

        _tf_broadcaster->sendTransform(map_to_odom);
        _tf_broadcaster->sendTransform(odom_to_base);

        _last_published = now;
        break;
      }
    }
  }

  gz::transport::Node _gz_node;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  double _pub_frequency;
  rclcpp::Time _last_published{0, 0, RCL_ROS_TIME};
  std::string _world_name;
  std::string _robot_name;
  std::string _base_link;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
