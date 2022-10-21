/**
 * @brief Node to setup a ghost car to be used for testing lane manuevers with a real car
 **/

// Application
#include <bvs_utils/heartbeat.h>
#include <bvs_utils/watchdog.h>
#include <bvs_utils/geodetic_conv.h>
#include <path_utils/path_util.h>

// ROS
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

const std::size_t execution_period_ms = 10;

namespace raceline_planner {

class GhostSim {
public:
    struct Params {
        double lookahead = 15.;
        double lookahead_noise_mean = 0.;
        double lookahead_noise_var = 0; //3;

        double velocity = 15.; // Debug Aadi - todo change this to 30
        double velocity_noise_mean = 0; // Debug Aadi - todo change this to -3.3
        double velocity_noise_var = 0; // Debug Aadi - todo change this to 1.5

        double track_noise_x = 0; //Debug Aadi - 1.5;
        double track_noise_y = 0; //Debug Aadi - 1.5;
        double track_noise_yaw = 0;//Debug Aadi - 0.5;
        double track_noise_vel = 0; //Debug Aadi - 1.4;
    };

    GhostSim():
        distribution_(0,1)
    {};

    Params& params() {
        return params_;
    }

    void setParams(Params& params) {
        params_ = params;
    }

    void initOdom(double x, double y) {
        cur_odom.pose.pose.position.x = x;
        cur_odom.pose.pose.position.y = y;
    }

    void setPath(const std::shared_ptr<path_utils::PathUtil>& path) {
        path_ = path;
    }

    std::shared_ptr<path_utils::PathUtil> getPath(){
        return path_;
    }
    void setPath(const nav_msgs::msg::Path& path) {
        path_ = nullptr;
        disc_path_ = path;
        disc_path_idx_ = 0;
    }

    nav_msgs::msg::Odometry step(double dt) {
        auto xn = cur_odom.pose.pose.position.x;
        auto yn = cur_odom.pose.pose.position.y;
        auto xf = 0.0;
        auto yf = 0.0;
        if(path_) {
            disc_path_ = path_->wrapPathTo(cur_odom.pose.pose);
            disc_path_idx_ = 0;
        } else if(disc_path_.poses.size() < 1) {
            return cur_odom;
        }
        double ln_now = params_.lookahead
            + distribution_(generator_) * params_.lookahead_noise_var
            + params_.lookahead_noise_mean;
        for(std::size_t i = disc_path_idx_; i < disc_path_.poses.size(); ++i) {
            double dist = std::sqrt(
                std::pow(disc_path_.poses[i].pose.position.x - xn, 2.0)
                + std::pow(disc_path_.poses[i].pose.position.y - yn, 2.0)
            );
            disc_path_idx_ = i;
            if(dist >= ln_now) {
                xf = disc_path_.poses[i].pose.position.x;
                yf = disc_path_.poses[i].pose.position.y;
                break;
            }
        }
        auto vx = xf - xn;
        auto vy = yf - yn;
        auto norm = std::sqrt(vx * vx + vy * vy);
        auto vel_now = params_.velocity
            + distribution_(generator_) * params_.velocity_noise_var
            + params_.velocity_noise_mean;
        auto yaw = std::atan2(vy, vx);
        auto mx = dt * vel_now * vx / norm;
        auto my = dt * vel_now * vy / norm;

        cur_odom.pose.pose.position.x += mx;
        cur_odom.pose.pose.position.y += my;
        cur_odom.pose.pose.position.z = 0;
        cur_odom.pose.pose.orientation.x = 0;
        cur_odom.pose.pose.orientation.y = 0;
        cur_odom.pose.pose.orientation.z = std::sin(yaw/2.);
        cur_odom.pose.pose.orientation.w = std::cos(yaw/2.);
        lx_ = cur_odom.pose.pose.position.x;
        ly_ = cur_odom.pose.pose.position.y;
        lyaw_ = yaw;
        return cur_odom;
    }

    bvs_msgs::msg::TrackedAgent lastTrack() {
        bvs_msgs::msg::TrackedAgent m;
        m.header.frame_id = "ltp";
        m.pose.position.x = lx_ + distribution_(generator_) * params_.track_noise_x;
        m.pose.position.y = ly_ + distribution_(generator_) * params_.track_noise_y;
        m.rpy.z = lyaw_ + distribution_(generator_) * params_.track_noise_yaw;
        m.velocity.x = params_.velocity + distribution_(generator_) * params_.track_noise_vel;
        m.speed = params_.velocity + distribution_(generator_) * params_.track_noise_vel;
        return m;
    }

private:
    double lx_, ly_, lyaw_;

    nav_msgs::msg::Odometry cur_odom;
    std::size_t disc_path_idx_ = 0;
    nav_msgs::msg::Path disc_path_;
    std::shared_ptr<path_utils::PathUtil> path_;
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> distribution_;
    Params params_;

};

class GhostNode : public rclcpp::Node {
public:
    GhostNode() : Node("GhostNode") {
        /*
            Construtor:
            - Initialize subscribers and publishers
            - Set parameters
            - Initialize ghost sim
        */

        // QoS Policy
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.transient_local();

        this->declare_parameter("detection_prefix_opp", "ghost_opp");
        auto detection_prefix_opp = this->get_parameter("detection_prefix_opp").as_string();

        opponent_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                    detection_prefix_opp + "/traj", 1);
        opponent_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                    detection_prefix_opp + "/odom", 1);
        tracking_pub_ = this->create_publisher<bvs_msgs::msg::TrackedAgents>(
                        "tracked_agent", 1);

        // Subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 1,
            std::bind(&GhostNode::odometryCallback, this, std::placeholders::_1));

        /* Use only for testing
        keyboard_path_ind_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "raceline_planner/keyboard/opponent_state", 1,
            std::bind(&GhostNode::keyboardPathIndCallBack, this,
                    std::placeholders::_1));
        */

        race_line_left_sub_ = this->create_subscription<bvs_msgs::msg::RaceLine>(
            "map_executive/line/race_line/left", qos,
            std::bind(&GhostNode::raceLineLCB, this,
                    std::placeholders::_1));
        race_line_right_sub_ = this->create_subscription<bvs_msgs::msg::RaceLine>(
            "map_executive/line/race_line/right", qos,
            std::bind(&GhostNode::raceLineRCB, this,
                    std::placeholders::_1));

        // Set up Execution
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(execution_period_ms),
            std::bind(&GhostNode::execute, this));

        // Read In parameters
        this->declare_parameter("raceline_names", std::vector<std::string>());
        this->declare_parameter("opp_start_ltp_x", 535.2617); // Set these in launch file
        this->declare_parameter("opp_start_ltp_y", -127.68210); // Set these in launch file
        this->declare_parameter("opp_vel", 15.); // Set these in launch file
        this->declare_parameter("ghost_enable", false);
        this->declare_parameter("spawn_near_ego", false);
        this->declare_parameter("spawn_dist", 80.0);
        
        raceline_names_ = {"left", "right"};//this->get_parameter("raceline_names").as_string_array();

        current_path_ind_ = 0;
        num_paths_loaded_ = 0;

        auto empty_line = std::make_shared<path_utils::PathUtil>();
        for(std::size_t i=0; i < raceline_names_.size(); i++){
            paths_.push_back(empty_line);
        }

        ltp_frame_ = "ltp"; // Get frame from origin topic in the future


        auto opp_start_ltp_x = this->get_parameter("opp_start_ltp_x").as_double();
        auto opp_start_ltp_y = this->get_parameter("opp_start_ltp_y").as_double();
        opponent_sim_.initOdom(opp_start_ltp_x, opp_start_ltp_y);
        opponent_sim_.params().velocity = 15; // Debug Aadi - 30
        opponent_sim_.params().velocity_noise_mean = 0; // Debug Aadi - -0.4
        opponent_sim_.params().velocity_noise_var = 0; // Debug Aadi - 1.8
    } 

    void execute() {
        opponent_sim_.params().velocity = this->get_parameter("opp_vel").as_double();
        ghost_enable_ = this->get_parameter("ghost_enable").as_bool();
        spawn_near_ego_ = this->get_parameter("spawn_near_ego").as_bool();
        spawn_dist_ = this->get_parameter("spawn_dist").as_double();
        
        if(num_paths_loaded_ < static_cast<int>(raceline_names_.size())){
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Waiting for paths to be loaded by map_exec");
            return;
        }
    
        if(ghost_enable_ && !old_ghost_enable_){
            if(!spawn_near_ego_)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Spawning far from ego"); // Spawn furthest away from ego
                geometry_msgs::msg::Pose far_pose =  paths_[current_path_ind_]->getDistPose(paths_[current_path_ind_]->path_dist_/2.0, ego_odometry_.pose.pose);
                opponent_sim_.initOdom(far_pose.position.x, far_pose.position.y);
            }
            else{
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Spawning close to ego");
                geometry_msgs::msg::Pose spawn_pose =  paths_[current_path_ind_]->getDistPose(spawn_dist_, ego_odometry_.pose.pose);
                opponent_sim_.initOdom(spawn_pose.position.x, spawn_pose.position.y);
            }
        }

        if(ghost_enable_){
            //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Ghost enabled!");
            opponent_sim_.setPath(paths_[current_path_ind_]);
            opponent_path_pub_->publish(paths_[current_path_ind_]->getPath());

            auto opp_odom = opponent_sim_.step(static_cast<double>(execution_period_ms) / 1000.);
            opp_odom.header.frame_id = ltp_frame_; 
            opponent_odom_pub_->publish(opp_odom);

            if(ghost_detected_){
                bvs_msgs::msg::TrackedAgents track_msg;
                track_msg.agents.push_back(opponent_sim_.lastTrack());
                track_msg.header.frame_id = ltp_frame_;
                tracking_pub_->publish(track_msg);
            }
            else{
                auto agentVector = bvs_msgs::msg::TrackedAgents();
                tracking_pub_->publish(agentVector);
            }
        }
        else { // Publish empty agent info
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Ghost disabled!");
            auto opp_odom = nav_msgs::msg::Odometry();
            opp_odom.header.frame_id = ltp_frame_; 
            opponent_odom_pub_->publish(opp_odom);

            auto agentVector = bvs_msgs::msg::TrackedAgents();
            tracking_pub_->publish(agentVector);
        }
        old_ghost_enable_ = ghost_enable_;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    ego_odometry_ = *message;
    }

    /* Use only for testing
    void keyboardPathIndCallBack(const std_msgs::msg::Int16::SharedPtr message){
        if(message->data == 1){
            current_path_ind_ = 0; // Index is message->data - 1
            opponent_sim_.setPath(paths_[current_path_ind_]);
            ghost_detected_ = true;
        }
        else if(message->data == 2){
            current_path_ind_ = 1; // Index is message->data - 1
            opponent_sim_.setPath(paths_[current_path_ind_]);
            ghost_detected_ = true;
        }
        else if(message->data == 3){
            ghost_detected_ = false;
        }

    }
    */

    void raceLineLCB(bvs_msgs::msg::RaceLine::SharedPtr message) {
        auto raceline = std::make_shared<path_utils::PathUtil>();
        raceline->loadPath(message);
        RCLCPP_INFO(get_logger(), "Loaded %ld points for left race line",
                    raceline->getLine().line.poses.size());
        paths_[0] = raceline;
        num_paths_loaded_ += 1;
    }
    void raceLineRCB(bvs_msgs::msg::RaceLine::SharedPtr message) {
        auto raceline = std::make_shared<path_utils::PathUtil>();
        raceline->loadPath(message);
        RCLCPP_INFO(get_logger(), "Loaded %ld points for right race line",
                    raceline->getLine().line.poses.size());
        paths_[1] = raceline;
        num_paths_loaded_ += 1;
    }

private:
    bool ghost_enable_;
    bool ghost_detected_ = true;
    bool old_ghost_enable_ = false;
    int num_paths_loaded_;
    std::vector<std::string> raceline_names_;
    std::vector<std::string> robot_path_files_;
    unsigned int current_path_ind_;
    std::vector<std::shared_ptr<path_utils::PathUtil>> paths_;
    std::string ltp_frame_;
    nav_msgs::msg::Odometry ego_odometry_;
    bool spawn_near_ego_;
    double spawn_dist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr opponent_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opponent_path_pub_;
    rclcpp::Publisher<bvs_msgs::msg::TrackedAgents>::SharedPtr tracking_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    //rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr keyboard_path_ind_sub_;
    
    //map exec
    rclcpp::Subscription<bvs_msgs::msg::RaceLine>::SharedPtr race_line_left_sub_;
    rclcpp::Subscription<bvs_msgs::msg::RaceLine>::SharedPtr race_line_right_sub_;

    rclcpp::TimerBase::SharedPtr execution_timer_;

    GhostSim opponent_sim_;

}; /* class GhostNode */

} /* namespace raceline_planner */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raceline_planner::GhostNode>());
    rclcpp::shutdown();
    return 0;
}
