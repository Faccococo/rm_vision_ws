#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace RMSimulatorLink
{
class RMSimulatorLink : public rclcpp::Node
{
private:
    double yaw;
    double pitch;
    double timestamp_offset = 0;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

public:
    RMSimulatorLink(const rclcpp::NodeOptions & options) : Node("rm_simulator_link", options)
    {
        RCLCPP_INFO(get_logger(), "RMSimulatorLink has been started.");
        joint_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint", 10, std::bind(&RMSimulatorLink::joint_callback, this, std::placeholders::_1));
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received joint state message");

        yaw = msg->position[0];
        pitch = msg->position[1];
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q;
        q.setRPY(0, pitch, yaw);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster->sendTransform(t);
    }
};

}  // namespace RMSimulatorLinks