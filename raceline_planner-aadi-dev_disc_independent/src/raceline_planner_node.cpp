/**
 * @brief Node to switch intelligently between racelines
 **/

// Application
#include <bvs_utils/heartbeat.h>
#include <bvs_utils/watchdog.h>
#include <bvs_utils/geodetic_conv.h>
#include <path_utils/path_util.h>
#include <system_executive/race_control/race_control.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "bvs_msgs/msg/tracked_agents.hpp"
#include "bvs_msgs/msg/safety_status.hpp"
#include <bvs_utils/heartbeat.h>
#include "bvs_msgs/msg/race_line.hpp"

#include <vector>
#include <string>

constexpr double CUBED(double x) { return x * x * x; }
constexpr double POW4(double x) { return x * x * x * x; }
constexpr double POW5(double x) { return x * x * x * x * x; }


namespace raceline_planner {

enum class SystemMode : uint8_t {
    NONE = 0,
    DEFEND = 1,
    ATTACK_NOVER = 2,
    ATTACK_OVER = 3,
    YELLOW = 4,
    GREEN = 5
};

enum class Action : uint8_t {
    TRAIL = 0,
    AVOID = 1,
    MAINTAIN = 3,
    SAFE_MAINTAIN = 4,
    DEFENSE_MERGE = 5,
    ATTACK_MERGE = 6,
    NO_PUBLISH = 7
};

enum class Lane : uint8_t {
    HOLD = 0,
    TRANSITION = 1
};

class RacelinePlannerNode : public rclcpp::Node {
public:
    RacelinePlannerNode() : Node("RacelinePlannerNode") {
        /*
            Construtor:
            - Initialize subscribers and publishers
            - Set parameters
            - Initialize the merging raceline
        */

        // QoS Policy
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.transient_local();

        // Subscribers:
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 1,
            std::bind(&RacelinePlannerNode::odometryCallback, this,
                      std::placeholders::_1));
        detections_sub_ = this->create_subscription<bvs_msgs::msg::TrackedAgents>(
            "/tracked_agent", 1,
            std::bind(&RacelinePlannerNode::detectionsCallback, this,
                      std::placeholders::_1));
        safety_status_sub_ = this->create_subscription<bvs_msgs::msg::SafetyStatus>(
            "system_executive/safety_status", 1,
            std::bind(&RacelinePlannerNode::safetyCallback, this,
                      std::placeholders::_1));
        max_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "system_executive/max_speed", 1,
            std::bind(&RacelinePlannerNode::maxSpeedCallback, this,
                    std::placeholders::_1));
        
        /* Uncomment for testing
        keyboard_path_ind_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "raceline_planner/keyboard/ego_state", 1,
            std::bind(&RacelinePlannerNode::keyboardPathIndCallBack, this,
                    std::placeholders::_1)); // Remove
        */

        // Publishers:
        track_point_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "raceline_planner/track", 1);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                    "planner/spline_racer/path", 1);

        merged_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                    "raceline_planner/merged_path", 1);

        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                    "planner/spline_racer/desired_speed", 1);

        max_speed_ = 30.;
        accn_ = 5.;
        disc_param_ = 1.1; // Greater than max discretization of the two lines!!
        safety_dist_ = 20.; 
        trailing_distance_ = 40.;
        overtaking_speed_up_ = 5.;
        trailing_vel_shift_ = 6.;
        safe_cutoff_distance_ = 37.; // ToDo: makes this dependent on relative velocity
        line_switch_debounce_ = 3;
        line_switch_debounce_count_ = 3;
        switch_lookahead_ = 10;
        profile_smoother_ = 200.;
        frame_id_ = "ltp";

        this->declare_parameter("profile_smoother", profile_smoother_);// lookahead, used to be k = 0.003
        this->declare_parameter("start_path", "left");
        this->declare_parameter("raceline_names", std::vector<std::string>());
        this->declare_parameter("raceline_topics", std::vector<std::string>());
        this->declare_parameter("pitline_topics", std::vector<std::string>());

        // Set up Execution
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RacelinePlannerNode::execute, this));

        // Read In parameters
        auto start_path = this->get_parameter("start_path").as_string();
        raceline_names_ = this->get_parameter("raceline_names").as_string_array();
        raceline_topics = this->get_parameter("raceline_topics").as_string_array();
        pitline_topics = this->get_parameter("pitline_topics").as_string_array();
        paths_loaded_ = false;

        race_line_left_sub_ = this->create_subscription<bvs_msgs::msg::RaceLine>(
            "map_executive/line/race_line/left", qos,
            std::bind(&RacelinePlannerNode::raceLineLCB, this,
                    std::placeholders::_1));
        race_line_right_sub_ = this->create_subscription<bvs_msgs::msg::RaceLine>(
            "map_executive/line/race_line/right", qos,
            std::bind(&RacelinePlannerNode::raceLineRCB, this,
                    std::placeholders::_1));
        ltp_origin_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "map_executive/artifact/ltp_origin", qos,
            std::bind(&RacelinePlannerNode::ltpOriginCB, this,
                    std::placeholders::_1));

        current_path_ = 255;
        auto i = 0;
        for(const auto n : raceline_names_) {
            if(n == start_path) {
                current_path_ = i;
            }
            raceline_publisher = this->create_publisher<nav_msgs::msg::Path>(
                    "raceline_planner/" + raceline_names_[i] + "/raceline", 1);
            raceline_publishers_.push_back(raceline_publisher);
            //ToDO: Add pitline publishers

            ++i;
        }
        if(current_path_ == 255) {
            RCLCPP_ERROR(get_logger(), "START PATH NOT FOUND, defaulting to 0");
            current_path_ = 0;
        } else {
            RCLCPP_INFO(get_logger(), "Selected start path as %s", raceline_names_[current_path_].c_str());
        }
        home_path_ = current_path_;
        targetting_path_ = current_path_;
        requested_mode_ = SystemMode::NONE;

        merge_raceline_ = std::make_shared<path_utils::PathUtil>();
        // Initialize merge_raceline
        bvs_utils::GeodeticConverter::GeoRef ref;
        bvs_utils::GeodeticConverter conv;
        ref.latitude = ltp_lat_;
        ref.longitude = ltp_lon_;
        ref.altitude = 0.;
        conv.initializeReference(ref);
        
        merge_raceline_->setConverter(conv);
        merge_raceline_->getLine().line.header.frame_id = frame_id_;
        merge_raceline_->setConstVelProfile(max_speed_);

        num_paths_loaded_ = 0;

        auto empty_line = std::make_shared<path_utils::PathUtil>();
        for(std::size_t i=0; i < raceline_names_.size(); i++){
            racelines_.push_back(empty_line);
        }

    }

    void execute() {
        profile_smoother_ = this->get_parameter("profile_smoother").as_double();

        if(num_paths_loaded_ < static_cast<int>(raceline_names_.size()) && !paths_loaded_){
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Waiting for paths to be loaded by map_exec");
            return;
        }
        else{
            paths_loaded_ = true;
        }

        if (!heartbeat) {
            heartbeat =
            std::make_shared<bvs_utils::Heartbeat>(shared_from_this(), "planner/spline_racer");
        }

        for(std::size_t i = 0; i < raceline_names_.size(); ++i) {
            auto line = racelines_[i]->getPath();
            line.header.frame_id = frame_id_;
            raceline_publishers_[i]->publish(line);
        }

        auto current_pathutil = racelines_[current_path_];

        auto ego_idx = current_pathutil->getClosestIdx(current_odometry_.pose.pose); // Index of point on path which ego is closest to

        std::size_t opp_idx = ego_idx;
        bool opponent_detected = false;
        geometry_msgs::msg::Pose opp_pose;

        if(agents_.agents.size() > 0) {
            opponent_detected = true;
            opp_pose = agents_.agents[0].pose;
            opp_idx = current_pathutil->getClosestIdx(opp_pose); // Index of point on path which opponent is closest to
        }

        // Make sure opponent can be tailed (we aren't in front of them)
        bool can_trail_opponent = false;
        int index_distance = static_cast<int>(opp_idx) - static_cast<int>(ego_idx);
        if(opponent_detected && index_distance > 0 && index_distance < static_cast<int>(current_pathutil->getLine().line.poses.size()/2.0)) {
            can_trail_opponent = true;
        } else if(opponent_detected && index_distance < 0 && -index_distance > static_cast<int>(current_pathutil->getLine().line.poses.size()/2.0)) {
            can_trail_opponent = true;
        }
       
        double distance_to_opp = 2*trailing_distance_; // TODO: fixthis
        if(opponent_detected){
            // Actual distance to opponent along the path
            distance_to_opp = current_pathutil->getPathDist(opp_pose, current_odometry_.pose.pose);
            
            //distance_to_opp = std::sqrt(
            //    std::pow(opp_pose.position.x - current_odometry_.pose.pose.position.x, 2.)
            //    + std::pow(opp_pose.position.y - current_odometry_.pose.pose.position.y, 2.)
            //);
        }
        else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Opponent not detected! Dist to opp set to: %3.2f", distance_to_opp);
        }

        //auto cpt = current_path->getClosestPose(current_odometry_.pose.pose);
        //auto overtake_allowed = cpt.second == path_utils::PathType::TRACK; // ToDo: Find end point of overtake and chekc if it is on track as well
        auto overtake_allowed = true;

        std::size_t safest_path = 0;
        //std::size_t opponent_path = 0;
        auto dist = 0.;
        //auto min_dist = std::numeric_limits<double>::infinity();
        for(std::size_t i = 0; i < raceline_topics.size(); ++i) {
            auto cpt = racelines_[i]->getClosestPose(opp_pose);
            auto dist_now = std::sqrt(
                std::pow(cpt.first.position.x - opp_pose.position.x, 2.0)
                + std::pow(cpt.first.position.y - opp_pose.position.y, 2.0)
            );
            if(dist_now > dist ) {
                safest_path = i;
                dist = dist_now;
            }
            //if(dist_now < min_dist){
            //    min_dist = dist_now;
            //    opponent_path = i;
            //}
        }

        double attack_over_vel = max_speed_ + overtaking_speed_up_/2.; // Velocity for attack merge

        // TODO: drive this from ROS topics
        SystemMode current_mode = requested_mode_;
        Action current_action = getAction(current_mode, opponent_detected, overtake_allowed, can_trail_opponent, safest_path, distance_to_opp, attack_over_vel);

        auto path_and_vel = get_vel_path(current_action, opponent_detected, can_trail_opponent, safest_path, distance_to_opp, attack_over_vel);
        std::size_t path_request = path_and_vel.first;
        double velocity_goal = path_and_vel.second;

        // Do not command negative velocities
        if(velocity_goal < 0){
            velocity_goal = 0.;
        }

        auto profiled_dist = current_pathutil->setNewVelProfile(velocity_goal, accn_, current_odometry_.pose.pose);
        //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Profiled dist: %3.2f", profiled_dist);
        if(current_action != Action::NO_PUBLISH) {
            if(path_request != targetting_path_ && targetting_path_ == current_path_) {
                // Interpolation logic:
                if(line_switch_debounce_ <= 1) { // Performs interpolation every x (line_switch debounce (3 in this case)) number of points
                    merge_raceline_->getPath() = get_interpolation_path(path_request);
                    merge_raceline_->setConstVelProfile(velocity_goal); // Todo: initial test, use new vel profile instead!
                } else {
                    --line_switch_debounce_;
                }
            } else {
                line_switch_debounce_ = line_switch_debounce_count_;
            }

            // Publishing path and velocity:
            if(targetting_path_ != current_path_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "On merging path!!");
                // Track into new path
                auto i = merge_raceline_->getClosestIdx(current_odometry_.pose.pose);

                if(i > last_merge_idx_) {
                    current_path_ = targetting_path_;
                }
                auto p = merge_raceline_->wrapPathTo(current_odometry_.pose.pose);
                p.header.frame_id = frame_id_;
                path_pub_->publish(p);
                std_msgs::msg::Float64 target_vel_msg;
                target_vel_msg.data = merge_raceline_->velocity_profile_[i]; // velocity_goal
                velocity_pub_->publish(target_vel_msg);

                merged_path_pub_->publish(p); // Publish merged path for visualization and debugging
            } else {
                auto p = current_pathutil->wrapPathTo(current_odometry_.pose.pose);
                p.header.frame_id = frame_id_;
                path_pub_->publish(p);
                std_msgs::msg::Float64 target_vel_msg;
                target_vel_msg.data =  current_pathutil->velocity_profile_[ego_idx];// velocity_goal;
                velocity_pub_->publish(target_vel_msg);
            }
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "No publish??????");
        }
    }

    Action getAction(SystemMode current_mode,
                     bool opponent_detected,
                     bool overtake_allowed,
                     bool can_trail_opponent,
                     std::size_t safest_path,
                     double distance_to_opp,
                     double attack_over_vel){

        // Following code block sets the current action
        Action current_action = Action::SAFE_MAINTAIN;
        if(current_mode == SystemMode::DEFEND) {
            if(opponent_detected && distance_to_opp > safe_cutoff_distance_ && current_path_ != home_path_) { // Competiton rule:Defender (5): The car should move to and stay on its defensive racing line and set its speed according to the speed target for the round. 
                current_action = Action::DEFENSE_MERGE;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: DEFEND, DEFENSE_MERGE");
            } else if(opponent_detected) {
                current_action = Action::SAFE_MAINTAIN;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: DEFEND, SAFE_MAINTAIN");
            } else {
                current_action = Action::MAINTAIN;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: DEFEND, MAINTAIN");
            }
        } else if(current_mode == SystemMode::ATTACK_OVER) {
            if(overtake_allowed && opponent_detected) {
                
                auto attack_dist = profile_smoother_ + switch_lookahead_ + safety_dist_; // Distance needed for attack merge
                double opp_speed = agents_.agents[0].speed;
                auto opp_covered_dist = opp_speed * attack_dist / attack_over_vel; // Distance covered by opp during attack merge

                if(can_trail_opponent && current_path_!=safest_path && distance_to_opp < std::max(attack_dist - opp_covered_dist, trailing_distance_ + safety_dist_)) { //Aadi - Add other conditions for where we should switch to outer race line
                    current_action = Action::ATTACK_MERGE;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: ATTACK_OVER, ATTACK_MERGE");
                } else if((can_trail_opponent || distance_to_opp < safe_cutoff_distance_) && current_path_ == safest_path) {
                    current_action = Action::AVOID;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: ATTACK_OVER, AVOID");
                } else if(current_path_ != home_path_) {
                    current_action = Action::DEFENSE_MERGE;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: ATTACK_OVER, DEFENSE_MERGE");
                } else if(can_trail_opponent) {
                    current_action = Action::TRAIL;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: ATTACK_OVER, TRAIL");
                } else {
                    current_action = Action::SAFE_MAINTAIN;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: ATTACK_OVER, SAFE_MAINTAIN");
                }
            } else {
                current_action = Action::TRAIL;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Overtake not allowed or opponnet not detected, trailll");
            }
        } else if(current_mode == SystemMode::ATTACK_NOVER) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: ATTACK_NOVER, TRAIL");
            current_action = Action::TRAIL;
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "State: GREEN or YELLOW or WHATEVS");
            //if(opponent_detected && distance_to_opp > safe_cutoff_distance_ && current_path_ != home_path_) { // When on green or yelow, it will default merge to home path
            //    current_action = Action::DEFENSE_MERGE;
            //} else if(opponent_detected) {
            if(opponent_detected) {
                current_action = Action::SAFE_MAINTAIN;
            } else {
                current_action = Action::MAINTAIN;
            }
        }

        return current_action;
    }

    std::pair<std::size_t, double> get_vel_path(Action current_action,
                                                bool opponent_detected,
                                                bool can_trail_opponent,
                                                std::size_t safest_path,
                                                double distance_to_opp,
                                                double attack_over_vel){
        // Exectue a given action primitive i.e. find path_request and velocity_goal
        std::size_t path_request = current_path_;
        double velocity_goal = max_speed_;
        // Actually execute a given primitive action
        if(current_action == Action::TRAIL) {
            //ToDo: Add derivative term (relative velocity ratio) to trailing logic
            //double rel_velocity_ratio = (max_speed_ - agents_.agents[0].velocity.x)/max_speed_; // Instead of max_speed, use ego's current speed
            double opp_speed = agents_.agents[0].speed;
            double rel_time = std::abs(opp_speed - max_speed_ - trailing_vel_shift_) / accn_; // Time to reach from current velocity to agent's velocity
            double rel_dist = opp_speed * rel_time; // Dist travelled by opp in that time
            double trail_shift_dist = std::abs(std::pow(opp_speed,2) - std::pow((max_speed_ + trailing_vel_shift_),2)) / (2*accn_) + trailing_distance_ - rel_dist;
            if(!opponent_detected || !can_trail_opponent || distance_to_opp >  trail_shift_dist) { // 2. * trailing_distance_
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "FAR TRAIL %3.2f, Current path: %s, Safest path: %s", distance_to_opp, raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
                velocity_goal = max_speed_ + trailing_vel_shift_;
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "CLOSE TRAIL %3.2f, Current path: %s, Safest path: %s", distance_to_opp, raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
                auto eratio = (trailing_distance_ - distance_to_opp) / trailing_distance_;
                if(distance_to_opp < trailing_distance_) {
                    eratio = 4 * eratio; // Todo - change multiplier as it is arbitarily picked using testing
                }
                if(eratio < -1){
                    eratio = -1;
                }
                velocity_goal = opp_speed - trailing_vel_shift_ * eratio;
            }
        } else if(current_action == Action::SAFE_MAINTAIN) {
            if(!opponent_detected || !can_trail_opponent || distance_to_opp > 2. * trailing_distance_ || current_path_ == safest_path) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "FAR SAFE MAINTAIN %3.2f, Current path: %s, Safest path: %s", distance_to_opp, raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
                velocity_goal = max_speed_;
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "CLOSE SAFE MAINTAIN %3.2f, Current path: %s, Safest path: %s", distance_to_opp, raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
                auto eratio = (trailing_distance_ - distance_to_opp) / trailing_distance_;
                if(distance_to_opp < trailing_distance_) {
                    eratio = 4 * eratio;
                }
                double opp_speed = agents_.agents[0].speed;
                velocity_goal = std::min(opp_speed - trailing_vel_shift_ * eratio, max_speed_);
            }
        } else if(current_action == Action::ATTACK_MERGE){ // ToDo - add smarter velocity profiling here
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Switching from Current path: %s to Safest path: %s", raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
            path_request = safest_path;
            velocity_goal = attack_over_vel;
        }
        else if(current_action == Action::AVOID) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Avoiding by staying on safest path %s, %3.2f", raceline_names_[safest_path].c_str(), distance_to_opp); // Debug Aadi - was "Avoiding on path %s, %3.2f
            velocity_goal = max_speed_ + overtaking_speed_up_;
        }else if(current_action == Action::MAINTAIN) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "MAINTAIN %3.2f, Current path: %s, Safest path: %s", distance_to_opp, raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
            velocity_goal = max_speed_;
        } else if(current_action == Action::DEFENSE_MERGE) { // Todo - add smarter velocity profiling here
            velocity_goal = max_speed_ + overtaking_speed_up_;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "DEFENSE MERGE %3.2f, Current path: %s, Safest path: %s", distance_to_opp, raceline_names_[current_path_].c_str(), raceline_names_[safest_path].c_str());
            path_request = home_path_;
        }

        return {path_request, velocity_goal};
    }

    nav_msgs::msg::Path get_interpolation_path(std::size_t path_request){
        targetting_path_ = path_request;
        line_switch_debounce_ = line_switch_debounce_count_;

        // Perform interpolation to other line
        RCLCPP_INFO(get_logger(), "Interpolating from %s to %s", raceline_names_[current_path_].c_str(), raceline_names_[path_request].c_str());
        auto target_p = racelines_[path_request]->wrapPathTo(current_odometry_.pose.pose);
        auto current_p = racelines_[current_path_]->wrapPathTo(current_odometry_.pose.pose);
        
        auto target_p_util = std::make_shared<path_utils::PathUtil>();
        target_p_util->loadPath(target_p);

        auto current_p_util = std::make_shared<path_utils::PathUtil>();
        current_p_util->loadPath(current_p);

        nav_msgs::msg::Path merged_path; 

        auto la_satis = false;
        for(std::size_t i = 0; i < target_p.poses.size() - 1 && i < current_p.poses.size() - 1; ++i) {
            double dist = disc_param_ * i;

            auto target_pose = target_p_util->getDistPose(dist, current_odometry_.pose.pose);
            auto current_pose = current_p_util->getDistPose(dist, current_odometry_.pose.pose);

            if(la_satis) {
                geometry_msgs::msg::PoseStamped merged_pose;
                double dist_ratio = (dist - switch_lookahead_) / profile_smoother_;
                if (dist_ratio >= 1.) {
                    break;
                }
                auto w_new = 6 * POW5(dist_ratio) - 15 * POW4(dist_ratio) + 10 * CUBED(dist_ratio);
                auto w_current = 1. - w_new;
                if(w_current <= 0. || w_new >= 1.) {
                    break;
                }

                auto tx = target_pose.position.x;
                auto ty = target_pose.position.y;
                auto cx = current_pose.position.x;
                auto cy = current_pose.position.y;


                merged_pose.pose.position.x = w_current*cx + w_new*tx;
                merged_pose.pose.position.y = w_current*cy + w_new*ty;

                auto len = merged_path.poses.size();
                auto new_yaw = std::atan2(
                    merged_pose.pose.position.y - merged_path.poses[len-1].pose.position.y,
                    merged_pose.pose.position.x - merged_path.poses[len-1].pose.position.x
                );

                merged_pose.pose.orientation.x = 0;
                merged_pose.pose.orientation.y = 0;
                merged_pose.pose.orientation.z = std::sin(new_yaw/2.);
                merged_pose.pose.orientation.w = std::cos(new_yaw/2.);

                merged_pose.header.frame_id = frame_id_;
                
                merged_path.poses.push_back(merged_pose);
            } else {
                if(dist > switch_lookahead_) { // ASSUMPTION: switch_lookahead always > targetp's discretized dist!!!
                    la_satis = true;
                }
                geometry_msgs::msg::PoseStamped merged_pose;
                merged_pose.pose = current_pose;
                merged_pose.header.frame_id = frame_id_;
                merged_path.poses.push_back(merged_pose); // current_pose
            }

        }

        // Combine merged path with target_p vector here!!
        nav_msgs::msg::Path combine_merged_target_p;
        auto len = merged_path.poses.size();
        auto target_p_ext = racelines_[path_request]->wrapPathTo(merged_path.poses[len-1].pose);
        combine_merged_target_p.poses.reserve( combine_merged_target_p.poses.size() + target_p_ext.poses.size()/2.0 ); // preallocate memory
        combine_merged_target_p.poses.insert( combine_merged_target_p.poses.end(), merged_path.poses.begin(), merged_path.poses.end() );
        combine_merged_target_p.poses.insert( combine_merged_target_p.poses.end(), target_p_ext.poses.begin(), target_p_ext.poses.begin() + target_p_ext.poses.size()/2.0); // Todo: find a better last pose
        combine_merged_target_p.header.frame_id = frame_id_;

        last_merge_idx_ = len-1;

        return combine_merged_target_p;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
        odom_update_time_ = rclcpp::Clock().now();
        current_odometry_ = *message;
    }

    void detectionsCallback(const bvs_msgs::msg::TrackedAgents::SharedPtr message) {
        agents_= *message;
    }


    void safetyCallback(const bvs_msgs::msg::SafetyStatus::SharedPtr message) {
        stateSelect(message->race_control_state);
    }

    void stateSelect(uint8_t state) {
        if(state == static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::DEFENDER)) {
            requested_mode_ = SystemMode::DEFEND;
        } else if(state == static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ATTACKER_NO_OVERTAKE)) {
            requested_mode_ = SystemMode::ATTACK_NOVER;
        } else if(state == static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ATTACKER_OVERTAKE)) {
            requested_mode_ = SystemMode::ATTACK_OVER;
        } else if(state == static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ON_TRACK_YELLOW)) {
            requested_mode_ = SystemMode::YELLOW;
        } else if(state == static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ON_TRACK_GREEN)) {
            requested_mode_ = SystemMode::GREEN;
        } else {
            requested_mode_ = SystemMode::NONE;
        }
    }

    void raceLineLCB(bvs_msgs::msg::RaceLine::SharedPtr message) {
        auto raceline = std::make_shared<path_utils::PathUtil>();
        raceline->loadPath(message);
        raceline->setConstVelProfile(max_speed_);
        RCLCPP_INFO(get_logger(), "Loaded %ld points for left race line",
                    raceline->getLine().line.poses.size());
        racelines_[0] = raceline;
        num_paths_loaded_ += 1;
    }
    void raceLineRCB(bvs_msgs::msg::RaceLine::SharedPtr message) {
        auto raceline = std::make_shared<path_utils::PathUtil>();
        raceline->loadPath(message);
        raceline->setConstVelProfile(max_speed_);
        RCLCPP_INFO(get_logger(), "Loaded %ld points for right race line",
                    raceline->getLine().line.poses.size());
        racelines_[1] = raceline;
        num_paths_loaded_ += 1;
    }

    void ltpOriginCB(geometry_msgs::msg::PointStamped message){
        ltp_lat_ = message.point.x;
        ltp_lon_ = message.point.y;
        frame_id_ = message.header.frame_id;
    }   

    void maxSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        max_speed_ = msg->data;
    }

    /* Uncomment for testing
    void keyboardPathIndCallBack(const std_msgs::msg::Int16::SharedPtr message){ // Remove
        if(message->data == 1){
            stateSelect(static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::DEFENDER)); // Defend
        }
        else if(message->data == 2){
            stateSelect(static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ATTACKER_NO_OVERTAKE)); // Attack No overtake
        }
        else if(message->data == 3){
            stateSelect(static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ATTACKER_OVERTAKE)); //Attack Overtake
        }
        else{
            stateSelect(static_cast<uint8_t>(system_executive::race_control::RaceControl::RaceControlStates::ON_TRACK_GREEN)); // Greens
        }

    }
    */
private:
    //rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr keyboard_path_ind_sub_; // Only for testing
    bool paths_loaded_;
    int num_paths_loaded_;
    std::vector<std::string> raceline_names_;
    std::vector<std::string> raceline_topics;
    std::vector<std::string> pitline_topics;
    double ltp_lat_;
    double ltp_lon_;

    std::shared_ptr<path_utils::PathUtil> merge_raceline_;
    std::shared_ptr<path_utils::PathUtil> raceline;
    std::shared_ptr<path_utils::PathUtil> pitline;

    std::vector<std::shared_ptr<path_utils::PathUtil>> racelines_;
    std::vector<std::shared_ptr<path_utils::PathUtil>> pitlines_;

    std::string frame_id_;
    std::size_t current_path_;
    std::size_t home_path_;
    std::size_t targetting_path_;
    std::size_t line_switch_debounce_;
    std::size_t line_switch_debounce_count_;

    std::size_t last_merge_idx_;
    bool overtake_init_ = false;
    SystemMode requested_mode_;

    bvs_msgs::msg::TrackedAgents agents_;
    nav_msgs::msg::Odometry current_odometry_;
    rclcpp::Time odom_update_time_;
    double max_speed_;
    double accn_;
    double disc_param_;
    double safety_dist_;
    double overtaking_speed_up_;
    double trailing_distance_;
    double trailing_vel_shift_;
    double safe_cutoff_distance_;
    double lateral_merge_rate_;
    double profile_smoother_;
    double switch_lookahead_;

    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> raceline_publishers_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raceline_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<bvs_msgs::msg::SafetyStatus>::SharedPtr safety_status_sub_;
    rclcpp::Subscription<bvs_msgs::msg::TrackedAgents>::SharedPtr detections_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_override_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr max_speed_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr track_point_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr merged_path_pub_;

    // map exec
    rclcpp::Subscription<bvs_msgs::msg::RaceLine>::SharedPtr race_line_left_sub_;
    rclcpp::Subscription<bvs_msgs::msg::RaceLine>::SharedPtr race_line_right_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ltp_origin_sub;

    rclcpp::TimerBase::SharedPtr execution_timer_;
    std::shared_ptr<bvs_utils::Heartbeat> heartbeat;

}; /* class RacelinePlannerNode */

} /* namespace raceline_planner */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raceline_planner::RacelinePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
