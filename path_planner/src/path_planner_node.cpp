#include "rrt.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
//#include <gmapping/grid/map.h>


void odometryCallback(const nav_msgs::Odometry data);
void globalMapCallback(const nav_msgs::OccupancyGrid& data);

// Сообщение с путем
nav_msgs::Path current_path;
nav_msgs::Path previous_path;

float ROBOT_HEIGHT = 0.7;
float ROBOT_WIDTH = 0.5;
const float CURVATURE = 0.2;

//Текущее положение платформы
geometry_msgs::Pose2D currentPosition;
nav_msgs::OccupancyGrid globalMap;

bool isCameGlobalMap = false;
bool isCameOdom = false;

int main(int argc, char **argv){
  ros::init(argc, argv, "kuka_path_searcher_node");

  // Создание публикатора пути
  ros::NodeHandle l;
  ros::Publisher path_for_rviz_pub = l.advertise<nav_msgs::Path>("/path_rrt", 8);
  ros::Publisher path_for_control_pub = l.advertise<nav_msgs::Path>("/target_path", 8);
  ros::Subscriber global_map_sub = l.subscribe("/map", 8, globalMapCallback);
  ros::Subscriber odom_sub = l.subscribe("/odom", 8, odometryCallback);
  ros::Publisher global_map_pub = l.advertise<nav_msgs::OccupancyGrid>("/g_map", 8);


  geometry_msgs::Pose2D goal;
  goal.x = 3;
  goal.y = 1;
  goal.theta = 1.5;

  ros::Rate rate(100);
  bool isAllowProcess = true;
  while(ros::ok() && isAllowProcess){

    if(isCameOdom && isCameGlobalMap){
      //      isCameOdom = false;
      //      isCameGlobalMap = false;

      previous_path = current_path;
      // Запуск планировщика
      RRT* rrt = new RRT();
      current_path = rrt->Planning(currentPosition, goal, globalMap, CURVATURE, ROBOT_HEIGHT, ROBOT_WIDTH);
      delete rrt;

      if(current_path.poses.size()){
        if(current_path.poses.size() < previous_path.poses.size() - 10|| !previous_path.poses.size()){
          cout << "Path is " << current_path.poses.size() << endl;
          path_for_control_pub.publish(current_path);
        }
        else{
          cout << "Path is not found" << endl;
        }
      }
    }
    path_for_rviz_pub.publish(current_path);
    global_map_pub.publish(globalMap);

    ros::spinOnce();
    rate.sleep();
  }
}

void odometryCallback(const nav_msgs::Odometry data){
  // Получение угла поворота
  tf::Pose pose;
  tf::poseMsgToTF(data.pose.pose, pose);
  float yawAngle = tf::getYaw(pose.getRotation());

  // Начальные координаты
  currentPosition.x = data.pose.pose.position.x;
  currentPosition.y = data.pose.pose.position.y;
  currentPosition.theta = yawAngle;
  isCameOdom = true;
}

void globalMapCallback(const nav_msgs::OccupancyGrid& data){
  globalMap.info.resolution = data.info.resolution;
  globalMap.info.height = data.info.height;
  globalMap.info.width = data.info.width;
  globalMap.info.origin = data.info.origin;

  globalMap.data = data.data;

  for(int i = 0; i < globalMap.info.width; i++)
    for(int j = 0; j < globalMap.info.height; j++){
      //      GMapping::IntPoint p(i, globalMap.info.width * j);
      //      double occ=smap.cell(p);
      if((int)globalMap.data[globalMap.info.width * j + i] == -1) {
        globalMap.data[globalMap.info.width * j + i] = 50;
        //      cout << (int)globalMap.data[globalMap.info.width * j + i] << endl;
      }
      if((int)globalMap.data[globalMap.info.width * j + i] > 75) {
        globalMap.data[globalMap.info.width * j + i] = 100;
        //      cout << (int)globalMap.data[globalMap.info.width * j + i] << endl;
      }
      if((int)globalMap.data[globalMap.info.width * j + i] < 20) {
        globalMap.data[globalMap.info.width * j + i] = 0;
        //      cout << (int)globalMap.data[globalMap.info.width * j + i] << endl;
      }
    }
  isCameGlobalMap = true;
}
