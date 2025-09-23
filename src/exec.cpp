#include "exec.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace turtle_behavior
{
    // Initialize global stop flag used by posture subscriber and BT nodes
    std::atomic<bool> stop_requested(false);

    // Subscriber for Nav2 navigation goal status
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_subscription_;

    // Action client for sending /navigate_to_pose goals
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client;

    // Shared ROS2 node instance used by action client and subscribers
    rclcpp::Node::SharedPtr node_;

    // Holds the latest navigation goal status; used to check success/failure
    int status_msg = 0;

    // ---------------- ActionSubscriber ----------------

    ActionSubscriber::ActionSubscriber() : Node("actionsubscriber")
    {
        // Subscribe to Nav2 goal status topic to monitor navigation state
        action_status_subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            std::bind(&ActionSubscriber::status_callback, this, std::placeholders::_1));
    }

    void ActionSubscriber::status_callback(action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        if (!msg->status_list.empty()) {
            status_msg = msg->status_list.back().status;
            RCLCPP_INFO(this->get_logger(), "Received Action Status: %d", status_msg);
        }
    }

    // ---------------- PostureStatusSubscriber ----------------

    PostureStatusSubscriber::PostureStatusSubscriber() : Node("posture_status_subscriber")
    {
        // Subscribe to /posture_status to check human posture
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/posture_status", 10,
            std::bind(&PostureStatusSubscriber::postureCallback, this, std::placeholders::_1));
    }

    void PostureStatusSubscriber::postureCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // If data == 1 indicates laying down, set stop_requested to true
        if (msg->data == 1) {  // 1 = lying down
            stop_requested.store(true);
            RCLCPP_INFO(this->get_logger(), "Stop requested: posture_status == 1 (LYING DOWN)");
        } else {
            // Otherwise clear the stop request
            stop_requested.store(false);
        }
    }

    // ---------------- Use_nav2 Behavior Tree Action Node ----------------

    Use_nav2::Use_nav2(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {}

    BT::PortsList Use_nav2::providedPorts()
    {
        return { BT::InputPort("goal_x"),
                 BT::InputPort("goal_y"),
                 BT::InputPort("goal_z") };
    }

    BT::NodeStatus Use_nav2::onStart()
    {
        std::cout << "Use_nav2: Starting navigation" << std::endl;

        // Lazy initialize ROS2 node and action client
        if (!node_) {
            node_ = rclcpp::Node::make_shared("nav2_bt_client");
        }
        if (!action_client) {
            action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/navigate_to_pose");
        }

        // Wait up to 10 seconds for action server to become available
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            std::cout << "Navigate to pose action server not available" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Prepare navigation goal message with requested coordinates
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

        double goal_x, goal_y, goal_z;
        getInput("goal_x", goal_x);
        getInput("goal_y", goal_y);
        getInput("goal_z", goal_z);

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = node_->now();
        goal_msg.pose.pose.position.x = goal_x;
        goal_msg.pose.pose.position.y = goal_y;
        goal_msg.pose.pose.position.z = goal_z;

        // Use orientation as identity quaternion (no rotation)
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        std::cout << "Sending navigation goal: (" << goal_x << ", " << goal_y << ", " << goal_z << ")" << std::endl;

        // Send the goal asynchronously
        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "Failed to send goal" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Store the goal handle for possible cancellation
        goal_handle_ = goal_handle_future.get();
        if (!goal_handle_) {
            std::cout << "Goal was rejected by server" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus Use_nav2::onRunning()
    {
        // Spin ROS2 node to process action callbacks
        rclcpp::spin_some(node_);

        // If stop requested via posture status, cancel current goal immediately
        if (stop_requested.load()) {
            if (goal_handle_) {
                std::cout << "Stopping navigation due to stop request" << std::endl;
                auto cancel_future = action_client->async_cancel_goal(goal_handle_);
                rclcpp::spin_until_future_complete(node_, cancel_future);
                return BT::NodeStatus::FAILURE;
            }
        }

        // Evaluate navigation status codes
        // 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if (status_msg == 4) {
            std::cout << "Navigation succeeded!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else if (status_msg == 5 || status_msg == 6) {
            std::cout << "Navigation failed or canceled with status: " << status_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Navigation still in progress
        return BT::NodeStatus::RUNNING;
    }

    void Use_nav2::onHalted()
    {
        std::cout << "Use_nav2: Navigation halted" << std::endl;

        // Cancel goal if halted to stop robot immediately
        if (action_client && goal_handle_) {
            std::cout << "Cancelling goal on halt" << std::endl;
            auto cancel_future = action_client->async_cancel_goal(goal_handle_);
            rclcpp::spin_until_future_complete(node_, cancel_future);
        }
    }

    // ---------------- Wait Behavior Tree Action Node ----------------

    Wait::Wait(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {}

    BT::PortsList Wait::providedPorts()
    {
        return { BT::InputPort<double>("inp_sec") };
    }

    BT::NodeStatus Wait::onStart()
    {
        double wait_seconds = 5.0;
        getInput("inp_sec", wait_seconds);
        std::cout << "Wait action started - waiting for " << wait_seconds << " seconds" << std::endl;

        // Calculate completion time for wait duration
        _completion_time = std::chrono::system_clock::now()
                         + std::chrono::milliseconds(static_cast<int>(wait_seconds * 1000));

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus Wait::onRunning()
    {
        if (std::chrono::system_clock::now() >= _completion_time) {
            std::cout << "Wait action completed - time finished" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "Wait action running - still waiting..." << std::endl;
            return BT::NodeStatus::RUNNING;
        }
    }

    void Wait::onHalted()
    {
        std::cout << "Wait action halted" << std::endl;
    }

    // ---------------- CheckLayDown Behavior Tree Condition Node ----------------

    CheckLayDown::CheckLayDown(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {}

    BT::PortsList CheckLayDown::providedPorts()
    {
        return {};
    }

    BT::NodeStatus CheckLayDown::tick()
    {
        // Return FAILURE if stop requested, which causes reactive BT to halt navigation
        if (stop_requested.load()) {
            std::cout << "Laydown detected - stop condition active" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
} // namespace turtle_behavior

// ---------------- main() ----------------

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    // Register custom BT nodes
    factory.registerNodeType<turtle_behavior::Use_nav2>("Use_nav2");
    factory.registerNodeType<turtle_behavior::Wait>("Wait");
    factory.registerNodeType<turtle_behavior::CheckLayDown>("CheckLayDown");

    // Load the behavior tree XML file from the package share directory
    std::string package_path = ament_index_cpp::get_package_share_directory("turtlebot_bt");
    auto tree = factory.createTreeFromFile(package_path + "/bt_xml/bt_tree_groot.xml");

    // Spin subscriber nodes in separate threads to run asynchronously
    std::thread action_subscriber_thread([]() {
        rclcpp::spin(std::make_shared<turtle_behavior::ActionSubscriber>());
    });
    std::thread posture_status_thread([]() {
        rclcpp::spin(std::make_shared<turtle_behavior::PostureStatusSubscriber>());
    });

    std::cout << "Starting behavior tree execution..." << std::endl;

    // Tick the BT root node continuously until finished or halted
    tree.tickRootWhileRunning();

    // Join subscriber threads before shutdown
    action_subscriber_thread.join();
    posture_status_thread.join();

    std::cout << "Behavior tree execution completed" << std::endl;

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

