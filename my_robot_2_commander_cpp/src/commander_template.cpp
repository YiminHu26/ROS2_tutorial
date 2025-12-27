#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using namespace std::placeholders;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;

class Commander
{
    // We will use a class Commander to encapsulate the node and MoveIt related functionalities
    // So the class is not the node. The node is inside the class.
    // so instead of using e.g. this->create_subscription, we will use e.g. node_->create_subscription
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            arm_->setMaxVelocityScalingFactor(1.0);
            arm_->setMaxAccelerationScalingFactor(1.0);
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

            open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper", 10, std::bind(&Commander::openJointCallback, this, _1));  
            // Initialize subscription, it's also possible to create a service here
            // Callback function see below
            // "this" means to bind the callback function to this class instance
            // _1 means the first argument of the callback function. Placeholder
            
            joint_cmd_sub_ = node_->create_subscription<FloatArray>("joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));
            pose_cmd_sub_ = node_->create_subscription<PoseCmd>("pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));
    
        }
    
        void goToNamedTarget(const std::string &name)
        {
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(name); // here we can also check whether the name exists
            planAndExecute(arm_);
        }

        void goToJointTarget(const std::vector<double> &joints)
        {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joints);
            planAndExecute(arm_);
        }

        void goToPoseTarget(double x, double y, double z,
                           double roll, double pitch, double yaw, bool cartesian_path = false)
                           // By providing just 6 params, normal target
                           // By providing cartesian_path = true, we will use Cartesian path planning
        {
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q = q.normalize();
            
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();

            arm_->setStartStateToCurrentState();

            if (!cartesian_path)
            {
                arm_->setPoseTarget(target_pose);
                planAndExecute(arm_);
            }
            else
            {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);

                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);// 0.01 = eef_step (increment)
                
                if (fraction == 1)
                {
                    arm_->execute(trajectory);
                }

            }
        }

        void openGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_open"); // Name according to my_robot_2_moveit_config/config/my_robot_2.srdf
            planAndExecute(gripper_);
        }

        void closeGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("gripper_closed"); // Name according to my_robot_2_moveit_config/config/my_robot_2.srdf
            planAndExecute(gripper_);
        }

    private:
        
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                interface->execute(plan);
            }
        }

        void openJointCallback(const Bool &msg)  // Callback function for the subscription, here only true or false
        {
            if (msg.data)
            {
                openGripper();
            }
            else
            {
                closeGripper();
            }
        }

        void jointCmdCallback(const FloatArray &msg)
        {
            auto joints = msg.data;

            if (joints.size() == 6)
            {
                goToJointTarget(joints);
            }
        }

        void poseCmdCallback(const PoseCmd &msg)
        {
            goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
        }
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_; 
        rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
        rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander"); // This is where the node is created
    auto commander = Commander(node); // Pass the node to the class
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
