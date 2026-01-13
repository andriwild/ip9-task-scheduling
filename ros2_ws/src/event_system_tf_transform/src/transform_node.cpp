#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
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
    this->declare_parameter("world_name", "fhnw");
    this->declare_parameter("robot_name", "dingo");
    this->declare_parameter("base_frame", "base_link");

    m_pub_frequency = this->get_parameter("publish_frequency").as_double();
    m_world_name    = this->get_parameter("world_name").as_string();
    m_robot_name    = this->get_parameter("robot_name").as_string();
    m_base_link     = this->get_parameter("base_frame").as_string();

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to Gazebo topic
    std::string topic = "/world/" + m_world_name + "/pose/info";

    if (!m_gz_node.Subscribe(topic, &GazeboTfBroadcaster::onPoseInfo, this)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to %s", topic.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo topic: %s", topic.c_str());
    }
  }

private:
  void onPoseInfo(const gz::msgs::Pose_V &msg) {
    auto now = this->now();

    // reduce publish frequency
    if (m_pub_frequency > 0.0 && m_last_published.seconds() > 0.0) {
      double elapsed = (now - m_last_published).seconds();
      if (elapsed < 1.0 / m_pub_frequency) {
        return;
      }
    }

    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto &pose = msg.pose(i);

      if (pose.name() == m_robot_name) {

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
        odom_to_base.child_frame_id = m_base_link;

        tf2::Quaternion gazebo_quat(
          pose.orientation().x(),
          pose.orientation().y(),
          pose.orientation().z(),
          pose.orientation().w()
        );

        odom_to_base.transform.translation.x = pose.position().x();
        odom_to_base.transform.translation.y = pose.position().y();
        odom_to_base.transform.translation.z = pose.position().z();

        odom_to_base.transform.rotation.w = pose.orientation().w();
        odom_to_base.transform.rotation.x = pose.orientation().x();
        odom_to_base.transform.rotation.y = pose.orientation().y();
        odom_to_base.transform.rotation.z = pose.orientation().z();

        m_tf_broadcaster->sendTransform(map_to_odom);
        m_tf_broadcaster->sendTransform(odom_to_base);

        m_last_published = now;
        break;
      }
    }
  }

  gz::transport::Node m_gz_node;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  double m_pub_frequency;
  rclcpp::Time m_last_published{0, 0, RCL_ROS_TIME};
  std::string m_world_name;
  std::string m_robot_name;
  std::string m_base_link;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
