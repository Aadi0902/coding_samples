/**
 * @brief Node to use direct keyboard inputs to switch between states for testing raceline planner
 **/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

class KeyboardSetStateNode : public rclcpp::Node
{
public:
    KeyboardSetStateNode() : Node("KeyboardSetStateNode")
    {
        RCLCPP_INFO(this->get_logger(), "Starting keyboard node:");
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&KeyboardSetStateNode::timerCallback, this));
        
        ego_state_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                    "raceline_planner/keyboard/ego_state", 1);
        opponent_state_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                    "raceline_planner/keyboard/opponent_state", 1);
        ego_state_.data = 1;
        opponent_state_.data = 1;
        
        ego_state_pub_->publish(ego_state_);
        opponent_state_pub_->publish(opponent_state_);
    }

private:
    void timerCallback()
    {
        std::string choice="";
        std::cout << "Choose state[1-5]" << std::endl
        << "1 - (EGO) DEFEND" << std::endl
        << "2 - (EGO) ATTACK_NOVER" << std::endl
        << "3 - (EGO) ATTACK_OVER" << std::endl
        << "4 - (OPPONENT) Inner_line" << std::endl 
        << "5 - (OPPONENT) Outer_line" << std::endl
        << "6 - (OPPONENT) Not detected" << std::endl
        << "Selection:";

        if(std::getline(std::cin, choice)){}
        

        if(choice=="1"){
            ego_state_.data = 1;
            std::cout << std::endl << "Ego set to defend!" << std::endl << std::endl;
        }
        else if(choice=="2"){
            ego_state_.data = 2;
            std::cout << std::endl << "Ego set to attack_nover!" << std::endl << std::endl;
        }
        else if(choice=="3"){
            ego_state_.data = 3;
            std::cout << std::endl << "Ego set to attack_over!" << std::endl << std::endl;
        }
        else if(choice=="4"){
            opponent_state_.data = 1;
            opponent_state_pub_->publish(opponent_state_);
            std::cout << std::endl << "Opponent set to inner_line!" << std::endl << std::endl;
        }
        else if(choice=="5"){
            opponent_state_.data = 2;
            opponent_state_pub_->publish(opponent_state_);
            std::cout << std::endl << "Opponent set to outer_line!" << std::endl << std::endl;
        }
        else if(choice=="6"){
            opponent_state_.data = 3;
            opponent_state_pub_->publish(opponent_state_);
            std::cout << std::endl << "Opponent not detected!" << std::endl << std::endl;
        }
        else{
            std::cout << "Invalid inputs" << std::endl;
        }
        ego_state_pub_->publish(ego_state_);
    }

    std_msgs::msg::Int16 ego_state_;
    std_msgs::msg::Int16 opponent_state_;

    rclcpp::TimerBase::SharedPtr execution_timer_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ego_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr opponent_state_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardSetStateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}