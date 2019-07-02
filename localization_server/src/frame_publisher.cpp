#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <math.h>

using namespace std;


geometry_msgs::TransformStamped createFrame(double x, double y, double z, int id){
    geometry_msgs::TransformStamped transformStamped;
    std::string _id = std::to_string(id);
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "marker_frame_"+_id;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(0,0,(0.5*M_PI));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    return transformStamped;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_publisher");
    ros::NodeHandle node;

    static tf2_ros::TransformBroadcaster br;
    vector<geometry_msgs::TransformStamped> marker_frames;
    for(int i = 0; i<1250; i++){
        int x = (i*1)%25;
        int y = floor(i/25);
        geometry_msgs::TransformStamped tS = createFrame((double)y/10.0, (double)x/10.0, 0, i);
        marker_frames.push_back(tS);
    }
    // geometry_msgs::TransformStamped tS0 = createFrame(0, 0, 0, 2);
    // marker_frames.push_back(tS0);
    // geometry_msgs::TransformStamped tS1 = createFrame(0, 0.1, 0, 1);
    // marker_frames.push_back(tS1);
    // geometry_msgs::TransformStamped tS2 = createFrame(0, 0.2, 0, 0);
    // marker_frames.push_back(tS2);
    // geometry_msgs::TransformStamped tS3 = createFrame(0.1, 0, 0, 5);
    // marker_frames.push_back(tS3);
    // geometry_msgs::TransformStamped tS4 = createFrame(0.1, 0.1, 0, 4);
    // marker_frames.push_back(tS4);
    // geometry_msgs::TransformStamped tS5 = createFrame(0.1, 0.2, 0, 3);
    // marker_frames.push_back(tS5);
    // geometry_msgs::TransformStamped tS6 = createFrame(0.2, 0, 0, 8);
    // marker_frames.push_back(tS6);
    // geometry_msgs::TransformStamped tS7 = createFrame(0.2, 0.1, 0, 7);
    // marker_frames.push_back(tS7);
    // geometry_msgs::TransformStamped tS8 = createFrame(0.2, 0.2, 0, 6);
    // marker_frames.push_back(tS8);

    ros::Rate rate(10.0);
    while(node.ok()){
        for(int i = 0; i<marker_frames.size(); i++){
            marker_frames[i].header.stamp = ros::Time::now();
            br.sendTransform(marker_frames[i]);
        }
        rate.sleep();
    }
    return 0;
}
