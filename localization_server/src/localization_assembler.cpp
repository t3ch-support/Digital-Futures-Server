#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <string>
using namespace std;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_assembler");
    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tL(tfBuffer);

    // Lists
    vector<ros::Publisher> locationPublishers;

    for(int i = 0; i < 21; i++){
        std::string id = std::to_string(i);
        ros::Publisher temp_robot_publisher;
        temp_robot_publisher = node.advertise<geometry_msgs::PoseStamped>("/digital_futures/robot_"+id+"/robot_global_pose", 10);
        locationPublishers.push_back(temp_robot_publisher);
    }
    ros::Publisher pose_array_publisher = node.advertise<geometry_msgs::PoseArray>("/digital_futures/poseArray/robot_global_poses", 10);
    

    vector<geometry_msgs::Pose> robotPoseHistory;
    for(int i = 0; i < 21; i++){
        geometry_msgs::Pose tempPose;
        tempPose.position.x = 0;
        tempPose.position.y = 0;
        tempPose.position.z = 0;
        tempPose.orientation.w = 0;
        tempPose.orientation.x = 0;
        tempPose.orientation.y = 0;
        tempPose.orientation.z = 0;
        robotPoseHistory.push_back(tempPose);
    }
    ros::Rate rate(30.0);
    while(node.ok()){
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = "map";
        poseArray.header.stamp = ros::Time::now();
        for(int i = 0; i < 21; i++){
            std::string id = std::to_string(i);
            bool failed = false;
            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("map", "robot_"+id, ros::Time(0));
            }catch(tf2::TransformException &ex){
                failed = true;
                //ROS_WARN("%s", ex.what());
                //ROS_WARN(failed);
                // ros::Duration(1.0).sleep();
                //continue;
            }
            if(failed){
                ROS_INFO_STREAM("Failed, pushing last known pose");
                poseArray.poses.push_back(robotPoseHistory[i]);
            }else{
                ROS_INFO_STREAM("Success");
                
                geometry_msgs::Pose pose;
                pose.position.x = transformStamped.transform.translation.x;
                pose.position.y = transformStamped.transform.translation.y;
                pose.position.z = transformStamped.transform.translation.z;
                pose.orientation.w = transformStamped.transform.rotation.w;
                pose.orientation.x = transformStamped.transform.rotation.x;
                pose.orientation.y = transformStamped.transform.rotation.y;
                pose.orientation.z = transformStamped.transform.rotation.z;
            
                poseArray.poses.push_back(pose);
                robotPoseHistory[i] = pose;
            }
            

            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.seq = transformStamped.header.seq;
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.header.frame_id = transformStamped.header.frame_id;
            poseStamped.pose.position.x = transformStamped.transform.translation.x;
            poseStamped.pose.position.y = transformStamped.transform.translation.y;
            poseStamped.pose.position.z = transformStamped.transform.translation.z;
            poseStamped.pose.orientation.w = transformStamped.transform.rotation.w;
            poseStamped.pose.orientation.x = transformStamped.transform.rotation.x;
            poseStamped.pose.orientation.y = transformStamped.transform.rotation.y;
            poseStamped.pose.orientation.z = transformStamped.transform.rotation.z;
            locationPublishers[i].publish(poseStamped);
            
        }
        pose_array_publisher.publish(poseArray);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
