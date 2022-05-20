#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>
#include "myworkcell_core/LocalizePart.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Localizer
{
    public:
        Localizer(ros::NodeHandle& nh) :
        listener_(buffer_)
        {
            ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
            &Localizer::visionCallback, this);
            server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
        }

        void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
        {
            last_msg_ = msg;
            // ROS_INFO_STREAM(last_msg_->pose.pose);
        } 

        bool localizePart(  myworkcell_core::LocalizePart::Request& req,
                            myworkcell_core::LocalizePart::Response& res)
        {
            fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
            if(!p) return false;

            geometry_msgs::PoseStamped target_pose_from_cam;
            target_pose_from_cam.header = p->header;
            target_pose_from_cam.pose = p->pose.pose;

            // ROS_INFO_STREAM("base_frame: " << req.base_frame);
            // ROS_INFO_STREAM("target_pose_from_cam: " << target_pose_from_cam.pose);

            geometry_msgs::PoseStamped target_pose_from_req = 
            buffer_.transform(target_pose_from_cam, req.base_frame);

            // ROS_INFO_STREAM("target_pose_from_req: " << target_pose_from_req);
            res.pose = target_pose_from_req.pose;

            return true;
        }

        ros::Subscriber ar_sub_;
        fake_ar_publisher::ARMarkerConstPtr last_msg_;
        ros::ServiceServer server_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        

};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    Localizer localizer(nh);
    ROS_INFO("Starting localizer vision node");
    ros::spin();
}