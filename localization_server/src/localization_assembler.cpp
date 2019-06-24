#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
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
    


    ros::Rate rate(30.0);
    while(node.ok()){
        for(int i = 0; i < 21; i++){
            std::string id = std::to_string(i);

            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("map", "robot_"+id, ros::Time(0));
            }catch(tf2::TransformException &ex){
                // ROS_WARN("%s", ex.what());
                // ros::Duration(1.0).sleep();
                continue;
            }
            //robot_publisher.publish(transformStamped);
            
            geometry_msgs::PoseStamped pose;
            pose.header.seq = transformStamped.header.seq;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = transformStamped.header.frame_id;
            pose.pose.position.x = transformStamped.transform.translation.x;
            pose.pose.position.y = transformStamped.transform.translation.y;
            pose.pose.position.z = transformStamped.transform.translation.z;
            pose.pose.orientation.w = transformStamped.transform.rotation.w;
            pose.pose.orientation.x = transformStamped.transform.rotation.x;
            pose.pose.orientation.y = transformStamped.transform.rotation.y;
            pose.pose.orientation.z = transformStamped.transform.rotation.z;
            locationPublishers[i].publish(pose);

        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
