#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// One row from goals.csv
struct GoalPose {
  double x;
  double y;
  double theta; // yaw in radians
};

class GoalPublisherNode : public rclcpp::Node
{
public:
  GoalPublisherNode()
  : Node("goal_publisher_node"), current_goal_index_(0)
  {
    // csv_file must be set by the launch file, no default makes sense
    this->declare_parameter<std::string>("csv_file", "");
    this->declare_parameter<double>("goal_timeout", 120.0);

    csv_file_     = this->get_parameter("csv_file").as_string();
    goal_timeout_ = this->get_parameter("goal_timeout").as_double();

    if (csv_file_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "csv_file parameter not set — cannot continue");
      rclcpp::shutdown();
      return;
    }

    // Create client for the navigate_to_pose action that Nav2 bt_navigator exposes
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Waiting for navigate_to_pose action server...");

    // Block until Nav2 is ready or 30s pass — this runs before spin() so it's synchronous
    if (!action_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_FATAL(this->get_logger(),
        "navigate_to_pose server not available after 30s — is Nav2 running?");
      rclcpp::shutdown();
      return;
    }

    if (!loadGoalsFromCSV(csv_file_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load goals from: %s", csv_file_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Loaded %zu goals from CSV. Sending first goal in 2s.", goals_.size());

    // Small delay so Nav2 finishes internal setup after the lifecycle manager activates it
    send_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() { this->sendNextGoal(); });
  }

private:
  bool loadGoalsFromCSV(const std::string & filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open: %s", filename.c_str());
      return false;
    }

    std::string line;
    std::getline(file, line); // skip the header line "x,y,theta"

    while (std::getline(file, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string cell;
      GoalPose gp;
      try {
        std::getline(ss, cell, ','); gp.x     = std::stod(cell);
        std::getline(ss, cell, ','); gp.y     = std::stod(cell);
        std::getline(ss, cell, ','); gp.theta = std::stod(cell);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(),
          "Skipping bad line '%s': %s", line.c_str(), e.what());
        continue;
      }
      goals_.push_back(gp);
    }
    return !goals_.empty();
  }

  // For 2D nav: roll=0, pitch=0, only yaw matters.
  // Formula: w=cos(yaw/2), z=sin(yaw/2), x=y=0.
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
  }

  void sendNextGoal()
  {
    // Cancel the timer so it doesn't fire again — we reschedule it in result_callback
    send_timer_->cancel();

    if (current_goal_index_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(),
        "All %zu goals completed. Shutting down.", goals_.size());
      rclcpp::shutdown();
      return;
    }

    const GoalPose & gp = goals_[current_goal_index_];
    RCLCPP_INFO(this->get_logger(),
      "Sending goal %zu/%zu: x=%.3f  y=%.3f  theta=%.3f rad",
      current_goal_index_ + 1, goals_.size(), gp.x, gp.y, gp.theta);

    // Nav2 requires goals expressed in the map frame
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id  = "map";
    goal_msg.pose.header.stamp     = this->now();
    goal_msg.pose.pose.position.x  = gp.x;
    goal_msg.pose.pose.position.y  = gp.y;
    goal_msg.pose.pose.position.z  = 0.0;
    goal_msg.pose.pose.orientation = yawToQuaternion(gp.theta);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Called when Nav2 accepts or rejects the goal request
    opts.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(),
            "Goal %zu rejected by server — advancing", current_goal_index_ + 1);
          current_goal_index_++;
          scheduleNext(1);
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal %zu accepted", current_goal_index_ + 1);
        }
      };

    // Called periodically while Nav2 is navigating — only log at DEBUG to avoid spam
    opts.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_DEBUG(this->get_logger(),
          "Goal %zu — %.2f m remaining", current_goal_index_ + 1, fb->distance_remaining);
      };

    // Called when navigation ends (success, abort, or cancel).
    // The spec says: send next goal when previous is reached OR aborted.
    // So we always advance regardless of result code.
    opts.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
              "Goal %zu SUCCEEDED", current_goal_index_ + 1);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(),
              "Goal %zu ABORTED — advancing", current_goal_index_ + 1);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),
              "Goal %zu CANCELED — advancing", current_goal_index_ + 1);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(),
              "Goal %zu unknown result", current_goal_index_ + 1);
            break;
        }
        current_goal_index_++;
        scheduleNext(1); // 1s pause between consecutive goals
      };

    action_client_->async_send_goal(goal_msg, opts);
  }

  void scheduleNext(int delay_sec)
  {
    send_timer_ = this->create_wall_timer(
      std::chrono::seconds(delay_sec),
      [this]() { this->sendNextGoal(); });
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  std::vector<GoalPose> goals_;
  size_t current_goal_index_;
  std::string csv_file_;
  double goal_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
