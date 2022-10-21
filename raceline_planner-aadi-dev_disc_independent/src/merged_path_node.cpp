/**
 * @brief Node to plot merged paths iteratively for raceline planner testing
 **/

// Application
#include <bvs_utils/heartbeat.h>
#include <bvs_utils/watchdog.h>
#include <bvs_utils/geodetic_conv.h>
#include <path_utils/path_util.h>

// ROS
#include <bvs_msgs/msg/safety_status.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "bvs_msgs/msg/tracked_agents.hpp"
#include "bvs_msgs/msg/tracked_agent.hpp"
#include "bvs_msgs/msg/race_line.hpp"

#include <vector>
#include <string>
#include <random>

const std::size_t execution_period_ms = 1700;

namespace raceline_planner {


class MergedPathNode : public rclcpp::Node {
public:
    MergedPathNode() : Node("MergedPathNode") {
        // QoS Policy
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.transient_local();

        this->declare_parameter("detection_prefix_ego", "");
        auto detection_prefix_ego = this->get_parameter("detection_prefix_ego").as_string();
        
        ego_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                    detection_prefix_ego + "odometry/filtered", 1);

        opp_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                    "dummy_opp/odom", 1);

        tracking_pub_ = this->create_publisher<bvs_msgs::msg::TrackedAgents>(
                    detection_prefix_ego + "tracked_agent", 1);

        safety_status_publisher =
        this->create_publisher<bvs_msgs::msg::SafetyStatus>(
            "system_executive/safety_status", 10);



        race_line_left_sub_ = this->create_subscription<bvs_msgs::msg::RaceLine>(
            "map_executive/line/race_line/left", qos,
            std::bind(&MergedPathNode::raceLineLCB, this,
                    std::placeholders::_1));
        race_line_right_sub_ = this->create_subscription<bvs_msgs::msg::RaceLine>(
            "map_executive/line/race_line/right", qos,
            std::bind(&MergedPathNode::raceLineRCB, this,
                    std::placeholders::_1));

        // Set up Execution
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(execution_period_ms),
            std::bind(&MergedPathNode::execute, this));

        // Read In parameters
        this->declare_parameter("ego_start_ltp_x", 535.2617544878366); // Set these in launch file 302.224042097263 - lor
        this->declare_parameter("ego_start_ltp_y", -127.68210278095052); // Set these in launch file 9130.685725510128 - lor
        this->declare_parameter("opp_spawn_dist", 100.); // Set these in launch file
        this->declare_parameter("ind_iter", 100); // Set these in launch file

        ego_path_ind_ = 0;
        paths_loaded_ = false;

        ego_start_ltp_x_ = this->get_parameter("ego_start_ltp_x").as_double();
        ego_start_ltp_y_ = this->get_parameter("ego_start_ltp_y").as_double();

        raceline_names_.push_back("left");
        raceline_names_.push_back("right");

        paths_loaded_ = false;
        ego_path_ind_ = 0;
        opp_path_ind_ = 0;

        opp_pose_ind_ = 0;
        ego_pose_ind_ = 0;
    } 

    void execute() {
        RCLCPP_INFO(get_logger(), "Running this!");
        opp_spawn_dist_ =  this->get_parameter("opp_spawn_dist").as_double();
        ind_iter_ =  this->get_parameter("ind_iter").as_int();

        nav_msgs::msg::Odometry ego_odom;
        nav_msgs::msg::Odometry opp_odom;
        if(paths_.size() < raceline_names_.size() && !paths_loaded_){
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Waiting for paths to be loaded by map_exec");
            return;
        }
        else if(!paths_loaded_){
            paths_loaded_ = true;
            ego_odom.pose.pose.position.x = ego_start_ltp_x_;
            ego_odom.pose.pose.position.y = ego_start_ltp_y_;

            ego_path_ = paths_[ego_path_ind_]->wrapPathTo(ego_odom.pose.pose);

            for(std::size_t ind = 1; ind < ego_path_.poses.size(); ind++){
                opp_odom.pose.pose = ego_path_.poses[ind].pose;
                if(paths_[ego_path_ind_]->getPathDist(ego_odom.pose.pose, opp_odom.pose.pose) > opp_spawn_dist_){ // fill this
                    break;
                }
            }

            opp_path_ = paths_[opp_path_ind_]->wrapPathTo(opp_odom.pose.pose);

            opp_pose_ind_ = 0;
            ego_pose_ind_ = 0;

        }

        bvs_msgs::msg::SafetyStatus safety_msg;
        safety_msg.race_control_state = 11; // Attack overtake
        safety_status_publisher->publish(safety_msg);


        bvs_msgs::msg::TrackedAgents track_msg;
        bvs_msgs::msg::TrackedAgent m;
        m.pose = opp_path_.poses[opp_pose_ind_].pose;
        m.header.frame_id = "ltp";
        track_msg.header.frame_id = "ltp";
        track_msg.agents.push_back(m);
        tracking_pub_->publish(track_msg);

        ego_odom.pose.pose = ego_path_.poses[ego_pose_ind_].pose;
        ego_odom.header.frame_id = "ltp";
        ego_odom_pub_->publish(ego_odom);

        opp_odom.pose.pose = opp_path_.poses[opp_pose_ind_].pose;
        opp_odom.header.frame_id = "ltp"; 
        opp_odom_pub_->publish(opp_odom);

        ego_pose_ind_ += ind_iter_;
        opp_pose_ind_ += ind_iter_;

        if(ego_pose_ind_ >= static_cast<int>(ego_path_.poses.size())){
            ego_pose_ind_ = static_cast<int>(ego_path_.poses.size()) - ego_pose_ind_ + ind_iter_;
        }
        if(opp_pose_ind_ >= static_cast<int>(opp_path_.poses.size())){
            opp_pose_ind_ = static_cast<int>(opp_path_.poses.size()) - opp_pose_ind_ + ind_iter_;
        }
    }

    void keyboardPathIndCallBack(const std_msgs::msg::Int16::SharedPtr message){
        if(message->data == 1){
            ego_path_ind_ = 0; // Index is message->data - 1
            opp_path_ind_ = 0;
        }
        else if(message->data == 2){
            ego_path_ind_ = 1; // Index is message->data - 1
            opp_path_ind_ = 1;
        }

    }

    void raceLineLCB(bvs_msgs::msg::RaceLine::SharedPtr message) {
        auto raceline = std::make_shared<path_utils::PathUtil>();
        raceline->loadPath(message);
        RCLCPP_INFO(get_logger(), "Loaded %ld points for left race line",
                    raceline->getLine().line.poses.size());
        paths_.push_back(raceline);
    }
    void raceLineRCB(bvs_msgs::msg::RaceLine::SharedPtr message) {
        auto raceline = std::make_shared<path_utils::PathUtil>();
        raceline->loadPath(message);
        RCLCPP_INFO(get_logger(), "Loaded %ld points for right race line",
                    raceline->getLine().line.poses.size());
        paths_.push_back(raceline);
    }

private:
    double ego_start_ltp_x_;
    double ego_start_ltp_y_;
    double opp_spawn_dist_;
    nav_msgs::msg::Path ego_path_;
    nav_msgs::msg::Path opp_path_;
    int ego_pose_ind_;
    int opp_pose_ind_;
    int ind_iter_;
    bool paths_loaded_;
    std::vector<std::string> raceline_names_;
    std::vector<std::string> robot_path_files_;
    int ego_path_ind_;
    int opp_path_ind_;
    std::vector<std::shared_ptr<path_utils::PathUtil>> paths_;
    std::string ltp_frame_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ego_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr opp_odom_pub_;
    rclcpp::Publisher<bvs_msgs::msg::TrackedAgents>::SharedPtr tracking_pub_;
    rclcpp::Publisher<bvs_msgs::msg::SafetyStatus>::SharedPtr
        safety_status_publisher;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ego_path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_vel_sub_;
    
    //map exec
    rclcpp::Subscription<bvs_msgs::msg::RaceLine>::SharedPtr race_line_left_sub_;
    rclcpp::Subscription<bvs_msgs::msg::RaceLine>::SharedPtr race_line_right_sub_;

    rclcpp::TimerBase::SharedPtr execution_timer_;

}; /* class MergedPathNode */

} /* namespace raceline_planner */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raceline_planner::MergedPathNode>());
    rclcpp::shutdown();
    return 0;
}
