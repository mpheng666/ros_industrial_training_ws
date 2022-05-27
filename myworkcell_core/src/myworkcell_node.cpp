#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <vector>
#include <algorithm>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <myworkcell_core/PlanCartesianPath.h>

class ScanNPlan
{
    public:
        ScanNPlan(ros::NodeHandle& nh) : 
        ac_("joint_trajectory_action", true)
        {
            vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
            cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
        }

        void start(const std::string& base_frame)
        {

            ros::AsyncSpinner async_spinner(1);
            async_spinner.start();
            
            ROS_INFO("Attempting to locate part");
            myworkcell_core::LocalizePart srv;

            srv.request.base_frame = base_frame;
            ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

            if(!vision_client_.call(srv))
            {
                ROS_ERROR("Could not localize part");
                return;
            }
            ROS_INFO_STREAM("part localized: " << srv.response);
            geometry_msgs::Pose move_target = srv.response.pose;
            moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
            moveit::planning_interface::MoveGroupInterface::Plan new_plan;
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

            // set all joints to zeros
            jointsCurrent_ = move_group.getCurrentJointValues();
            numOfJoints_ = jointsCurrent_.size();
            jointsAllZeros_.resize(numOfJoints_);
            std::fill(jointsAllZeros_.begin(), jointsAllZeros_.end(), 0.0);

            // Plan for robot to move to part
            move_group.setPoseReferenceFrame(base_frame);
            move_group.setPoseTarget(move_target);
            auto success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_STREAM("success: " << success);

            if(success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                // move_group.setPoseTarget();
                ROS_INFO("Moving all joints to zero!");
                move_group.setJointValueTarget(jointsAllZeros_);
                move_group.move();
            }

            // Plan cartesian path
            myworkcell_core::PlanCartesianPath cartesian_srv;
            cartesian_srv.request.pose = move_target;
            if (!cartesian_client_.call(cartesian_srv))
            {
            ROS_ERROR("Could not plan for path");
            return;
            }

            // Execute descartes-planned path directly (bypassing MoveIt)
            ROS_INFO("Got cart path, executing");
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = cartesian_srv.response.trajectory;
            ac_.sendGoal(goal);
            ac_.waitForResult();
            ROS_INFO("Done");
        }

    private:
        ros::ServiceClient vision_client_;
        std::vector<double> jointsCurrent_;
        std::vector<double> jointsTarget_;
        std::vector<double> jointsAllZeros_;

        int numOfJoints_ {6};

        const std::string PLANNING_GROUP = "manipulator";

        ros::ServiceClient cartesian_client_;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;


};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "myworkcell_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle ("~");

    std::string base_frame;
    private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

    ScanNPlan app(nh);
    ros::Duration(.5).sleep();
    app.start(base_frame);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    ros::waitForShutdown();
}