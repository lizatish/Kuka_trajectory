#include "rrt.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Polygon.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

void odometryCallback(const nav_msgs::Odometry data);
void formPathMessage();
void globalMapCallback(const nav_msgs::OccupancyGrid& data);
// Инициализация параметров карты и пути
void pathMessageInitParams();

// Сообщение с путем
nav_msgs::Path pathMessage;
vector<geometry_msgs::Point> path;

const float ROBOT_WIDTH_HALF = 0.6/2;
const float CURVATURE = 0.2;

// Размер карты
float mapResolution = 0;
int mapSize = 0;

//Текущее положение платформы
geometry_msgs::Point currentPosition;
// Угол поворота робота
float yawAngle;

nav_msgs::OccupancyGrid globalMap;
bool isCameGlobalMap = false;
bool isCameOdom = false;

geometry_msgs::Point pointToSend;
nav_msgs::Path target_path;

int main(int argc, char **argv){
  ros::init(argc, argv, "kuka_path_searcher_node");

  // Создание публикатора пути
  ros::NodeHandle l;
  ros::Publisher path_pub = l.advertise<nav_msgs::Path>("/path_rrt", 8);
  ros::Publisher target_path_pub = l.advertise<nav_msgs::Path>("/target_path", 8);
  ros::Publisher target_point_pab = l.advertise<geometry_msgs::Point> ("/target_point", 8);
  ros::Subscriber global_map_sub = l.subscribe("/global_map", 8, globalMapCallback);
  ros::Subscriber odom_sub = l.subscribe("/odom", 8, odometryCallback);

  pathMessageInitParams();

  geometry_msgs::Point goal;
  //      goal.x = 0.3;
  //      goal.y = 0;
  //      goal.z = -1.5;

  goal.x = 5;
  goal.y = 3;
  goal.z = -1.5;

  //    goal.x = 4;
  //    goal.y = 0.5;
  //    goal.z = 0;


  ros::Rate rate(100);
  bool isAllowProcess = true;
  ros::Time start_time = ros::Time::now();
  while(ros::ok() && isAllowProcess){

    if(ros::Time::now().toSec() - start_time.toSec() < 1)
      continue;

    if(!path.size()){
      if(isCameOdom && isCameGlobalMap){
        isCameOdom = false;
        isCameGlobalMap = false;

        // Запуск планировщика
        RRT* rrt = new RRT();
        path = rrt->Planning(currentPosition, goal, globalMap, CURVATURE, ROBOT_WIDTH_HALF);
        delete rrt;

        //        if(path.size() > 110){
        //          path.clear();
        //          continue;
        //        }
        if(path.size()){
          cout << "Path is found " << path.size() << endl;
          geometry_msgs::PoseStamped point;
          for (int k = path.size() - 1; k >= 0; k--){
            float nx = path[k].x;
            float ny = path[k].y;
            point.pose.position.x = nx - mapSize/2*mapResolution;
            point.pose.position.y = ny - mapSize/2*mapResolution;
            point.pose.position.z = path[k].z;
            target_path.poses.push_back(point);
          }
          target_path_pub.publish(target_path);
          formPathMessage();
        }
        else{
          cout << "Path is not found" << endl;
          target_path_pub.publish(target_path);
          formPathMessage();
        }
      }
    }
    else{
      vector<geometry_msgs::Point> path_for_check;
      RRT* rrt_check = new RRT();
      path_for_check = rrt_check->Planning(currentPosition, goal, globalMap, CURVATURE, ROBOT_WIDTH_HALF);
      delete rrt_check;

      if(path_for_check.size() && (path_for_check.size() + 20 < path.size())){
        cout << "Find shorter path, size "  << path_for_check.size() << endl;

        pathMessage.poses.clear();
        path.clear();
        target_path.poses.clear();

        path = path_for_check;
        geometry_msgs::PoseStamped point;
        for (int k = path.size() - 1; k >= 0; k--){
          float nx = path[k].x;
          float ny = path[k].y;
          point.pose.position.x = nx - mapSize/2*mapResolution;
          point.pose.position.y = ny - mapSize/2*mapResolution;
          point.pose.position.z = path[k].z;
          target_path.poses.push_back(point);
        }
        target_path_pub.publish(target_path);

        formPathMessage();
      }
      else{
        // Проверка на пересечение с новой картой
        for (int k = path.size() - 1; k >= 0; k--){
          int nx = path[k].x/mapResolution;
          int ny = path[k].y/mapResolution;
          if(globalMap.data[mapSize * ny + nx] > 75){
            pathMessage.poses.clear();
            path.clear();
            target_path.poses.clear();
            cout << "OBSTACLE! Search new path" << endl;
          }
        }
      }
      path_for_check.clear();
    }
    path_pub.publish(pathMessage);
    ros::spinOnce();
    rate.sleep();
  }
}

void odometryCallback(const nav_msgs::Odometry data){
  // Получение угла поворота
  tf::Pose pose;
  tf::poseMsgToTF(data.pose.pose, pose);
  yawAngle = tf::getYaw(pose.getRotation());

  // Окончательные начальные координаты
  currentPosition.x = data.pose.pose.position.x;
  currentPosition.y = data.pose.pose.position.y;
  currentPosition.z = yawAngle;
  //  cout << currentPosition.x << " " << currentPosition.y << " " << yawAngle << endl;
  isCameOdom = true;
}

void globalMapCallback(const nav_msgs::OccupancyGrid& data){
  mapResolution = data.info.resolution;
  mapSize = data.info.height;
  globalMap = data;
  isCameGlobalMap = true;
}

void formPathMessage(){
  // Точка для загрузки в сообщение пути
  geometry_msgs::PoseStamped point;
  for (int k = path.size() - 1; k >= 0; k--){
    float nx = path[k].x;
    float ny = path[k].y;

    point.pose.position.x = nx - mapSize/2*mapResolution;
    point.pose.position.y = ny - mapSize/2*mapResolution;
    pathMessage.poses.push_back(point);
  }
}

void pathMessageInitParams(){
  pathMessage.header.frame_id = "/odom";
  pathMessage.header.stamp = ros::Time::now();
}
