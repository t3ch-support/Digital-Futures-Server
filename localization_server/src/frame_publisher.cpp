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
        geometry_msgs::TransformStamped tS = createFrame((double)y/10.0, 2.5-(double)x/10.0, 0, i);
        marker_frames.push_back(tS);
    }


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
