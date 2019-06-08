#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odometryCallback(const nav_msgs::Odometry data);
void tf_base_link_to_laser();
void tf_odom_to_base_link();

bool isCameOdom = false;
// odom - base_link
geometry_msgs::TransformStamped odom_trans;
// base_link - laser
geometry_msgs::TransformStamped base_link_trans;

ros::Time current_time;
nav_msgs::Odometry cur_odom;

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/odom", 8, odometryCallback);
  tf::TransformBroadcaster odom2base_link_broadcaster;
  tf::TransformBroadcaster base_link2laser_broadcaster;

  current_time = ros::Time::now();
  ros::Rate r(20.0);
  while(n.ok()){

    ros::spinOnce();
    current_time = ros::Time::now();

    //send the transform
    if(isCameOdom){
      tf_odom_to_base_link();
      odom2base_link_broadcaster.sendTransform(odom_trans);
    }
    tf_base_link_to_laser();
    base_link2laser_broadcaster.sendTransform(base_link_trans);

    r.sleep();
  }
}
void odometryCallback(const nav_msgs::Odometry data){
  cur_odom = data;
  isCameOdom = true;
}
void tf_odom_to_base_link(){
  //transform over tf
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = cur_odom.pose.pose.position.x;
  odom_trans.transform.translation.y = cur_odom.pose.pose.position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = cur_odom.pose.pose.orientation;
}

void tf_base_link_to_laser(){
  //transform over tf
  base_link_trans.header.stamp = current_time;
  base_link_trans.header.frame_id = "base_link";
  base_link_trans.child_frame_id = "laser";

  base_link_trans.transform.translation.x = 0.24;
  base_link_trans.transform.translation.y = 0.0;
  base_link_trans.transform.translation.z = 0.0;
  geometry_msgs::Quaternion tf_quat = tf::createQuaternionMsgFromYaw(0);
  base_link_trans.transform.rotation = tf_quat;
}
