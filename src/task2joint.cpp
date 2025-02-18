// Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tutorial_interfaces/srv/set_target_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


// MoveGroupInterface
#include <moveit/move_group_interface/move_group_interface.hpp>

// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp 
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"

// Mode
const int INIT = 0;
const int TELEOP = 1;
const int TASK_CONTROL = 2;
const int JOINT_CONTROL = 3;
const int AI = 4;
const int MOVEIT = 5;
const int IDLE = 6;

// Node class
class Task2JointNode : public rclcpp::Node {
public:
    Task2JointNode(
        const std::string& test, 
        robot_model_loader::RobotModelLoader robot_model_loader,
        const moveit::core::RobotModelPtr& kinematic_model) 
    : Node("task2joint_node"), test(test), robot_model_loader(robot_model_loader), kinematic_model(kinematic_model)
    {
        const auto& LOGGER = this->get_logger();
        RCLCPP_INFO(LOGGER, "Task2Joint Node started");
        RCLCPP_INFO(LOGGER, "test: %s", test.c_str());

        // Custom move_group interface
        //PLANNING_GROUP = "ur10_arm";
        //moveit::planning_interface::MoveGroupInterface move_group(this, PLANNING_GROUP);
        RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());
        robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
        //robot_state->setToDefaultValues();
        joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
        //const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        joint_names = joint_model_group->getVariableNames();

        // Get Joint Values
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        // IK service
        // ik_service_ = this->create_service<tutorial_interfaces::srv::SetTargetPose>(
        //     "solve_ik",
        //     std::bind(&Task2JointNode::solve_ik, this, std::placeholders::_1, std::placeholders::_2)
        // );
        // set_target_pose_client_ = 
        //     this->create_client<tutorial_interfaces::srv::SetTargetPose>("set_target_pose");
        
        // Publishers
        ik_result_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ik_result", 10);
        ik_failed_pub_ = this->create_publisher<std_msgs::msg::Bool>("ik_failed", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Task2JointNode::joint_state_callback, this, std::placeholders::_1)
        );
        // Subscribers
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&Task2JointNode::target_pose_callback, this, std::placeholders::_1)
        );
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode", 10, std::bind(&Task2JointNode::mode_callback, this, std::placeholders::_1)
        );

        current_joint_position.assign(6, 0.0);
        current_joint_velocity.assign(6, 0.0);
        ik_result = std_msgs::msg::Float64MultiArray();
        pre_ik_result = std_msgs::msg::Float64MultiArray();
        ik_result.data = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0};
        pre_ik_result.data = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0};

        // 주기적 실행 타이머 (4ms마다 실행)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&Task2JointNode::control_loop, this)
        );
    }

private:
    // Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ik_failed_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    // 서비스 서버 & 클라이언트
    rclcpp::Service<tutorial_interfaces::srv::SetTargetPose>::SharedPtr ik_service_;
    rclcpp::Client<tutorial_interfaces::srv::SetTargetPose>::SharedPtr set_target_pose_client_;

    // 실행 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    const std::string PLANNING_GROUP = "ur10_arm";
    std::string test;
    robot_model_loader::RobotModelLoader robot_model_loader;
    const moveit::core::RobotModelPtr& kinematic_model;

    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;

    sensor_msgs::msg::JointState current_joint_state;
    geometry_msgs::msg::PoseStamped current_target_pose;
    std::vector<double> current_joint_position, current_joint_velocity, joint_values;
    std_msgs::msg::Float64MultiArray ik_result, pre_ik_result;

    int mode = INIT;
    double continuity_threshold = 0.3;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->current_joint_state = *msg;
        this->current_joint_position = current_joint_state.position;
        this->current_joint_velocity = current_joint_state.velocity;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", this->current_joint_state.name[0].c_str());
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f", this->current_joint_position[0], this->current_joint_velocity[0]);
    }

    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->current_target_pose = *msg;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", this->current_target_pose.pose.position.x);
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        this->mode = msg->data;
    }

    bool solve_ik(geometry_msgs::msg::Pose target_pose){
        // set joint position to current state
        robot_state->setJointGroupPositions(joint_model_group, current_joint_position);
        // enforce joint limits
        Eigen::Isometry3d pose_in;
        robot_state->enforceBounds();
        Eigen::fromMsg(target_pose, pose_in);
        
        bool found_ik = robot_state->setFromIK(joint_model_group, pose_in, 0.1);
        robot_state->update(); // https://github.com/ros-planning/moveit/pull/188
        
        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
            robot_state->copyJointGroupPositions(joint_model_group, joint_values);
            RCLCPP_INFO(this->get_logger(), "found IK solution");
            // publish result
            ik_result.data.clear();
            ik_result.data.push_back(joint_values[0]);
            ik_result.data.push_back(joint_values[1]);
            ik_result.data.push_back(joint_values[2]);
            ik_result.data.push_back(joint_values[3]);
            ik_result.data.push_back(joint_values[4]);
            ik_result.data.push_back(joint_values[5]);
            
            // Check solution continuity
            bool continuity = check_solution_continuity();
            if(!continuity){
                found_ik = false;
                // ROS_INFO("IK solution is not continuous");
                RCLCPP_INFO(this->get_logger(), "IK solution is not continuous");
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Did not find IK solution");
        }
        return found_ik;
    }

    bool check_solution_continuity(void){
        bool solution_continuity = true;
        for(int i=0; i<joint_names.size(); i++){
          if (abs(ik_result.data[i]-pre_ik_result.data[i]) > continuity_threshold)
            solution_continuity = false;
        }
        return solution_continuity;
    }

    void control_loop() {
        // RCLCPP_INFO(this->get_logger(), "Current mode: %d", mode);
        if (mode == TELEOP || mode == TASK_CONTROL || mode == AI) {
            RCLCPP_INFO(this->get_logger(), "Solve IK");
            bool success = solve_ik(this->current_target_pose.pose);
            if (success) {
                ik_result_pub_->publish(ik_result);
                std_msgs::msg::Bool ik_failed_msg;
                ik_failed_msg.data = false;
                ik_failed_pub_->publish(ik_failed_msg);
                pre_ik_result = ik_result;
            }
        } else if (mode == INIT) {
            // RCLCPP_INFO(this->get_logger(), "INIT");
            this->pre_ik_result.data = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0}; // init pose
        }

    }
};

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_cpp", node_options);
    static const std::string PLANNING_GROUP = "ur10_arm";
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP);
    robot_model_loader::RobotModelLoader robot_model_loader(move_group_node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    auto node = std::make_shared<Task2JointNode>("test", robot_model_loader, kinematic_model);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}