#ifndef TURTLEBOT_BT_INCLUDE_EXEC_HPP
#define TURTLEBOT_BT_INCLUDE_EXEC_HPP

#include <iostream>
#include <string>
#include <atomic>
#include <chrono>
#include <thread>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"

namespace turtle_behavior
{
    // Global atomic flag shared across nodes to indicate stop requested by posture status
    extern std::atomic<bool> stop_requested;

    /**
     * Subscriber node to track navigation goal status from Nav2.
     * Maintains the current status of the active navigation goal.
     */
    class ActionSubscriber : public rclcpp::Node
    {
    public:
        ActionSubscriber();

        // Callback for receiving Nav2 navigation status updates
        void status_callback(action_msgs::msg::GoalStatusArray::SharedPtr msg);
    };

    /**
     * Subscriber node for /posture_status topic to detect if human posture == laying down.
     * Sets stop_requested flag when posture indicates laying down (data == 1).
     */
    class PostureStatusSubscriber : public rclcpp::Node
    {
    public:
        PostureStatusSubscriber();

    private:
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

        // Callback to process posture status messages and update stop flag
        void postureCallback(const std_msgs::msg::Int32::SharedPtr msg);
    };

    /**
     * Behavior Tree action node that sends navigation goals to Nav2.
     * Uses ROS2 action client to send and manage /navigate_to_pose goals.
     * Can cancel goal immediately if stop_requested is true.
     */
    class Use_nav2 : public BT::StatefulActionNode
    {
    public:
        Use_nav2(const std::string& name, const BT::NodeConfiguration& config);

        // Define input ports for goal coordinates
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;    // Called once when node starts
        BT::NodeStatus onRunning() override;  // Called repeatedly while node is running
        void onHalted() override;              // Called on halt/interruption

    private:
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    };

    /**
     * Behavior Tree action node that waits for a specified duration.
     * Demonstrates a simple wait with configurable input seconds.
     */
    class Wait : public BT::StatefulActionNode
    {
    public:
        Wait(const std::string& name, const BT::NodeConfiguration& config);

        // Define input port for wait duration in seconds
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        std::chrono::system_clock::time_point _completion_time;  // Target time to finish waiting
    };

    /**
     * Behavior Tree condition node checking for laying-down posture.
     * Returns FAILURE if stop_requested is true, else SUCCESS,
     * to control reactive sequence in BT for emergency stop.
     */
    class CheckLayDown : public BT::ConditionNode
    {
    public:
        CheckLayDown(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts();

        // Tick method called each BT evaluation cycle
        BT::NodeStatus tick() override;
    };
}

#endif // TURTLEBOT_BT_INCLUDE_EXEC_HPP

