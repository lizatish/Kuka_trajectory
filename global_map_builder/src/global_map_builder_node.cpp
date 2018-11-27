///*           Описание модуля
// Модуль предназначен для получения и обработки
// локальной карты и одометрии, и
// соединения локальной и глобальной карт
///*

#include <ros/ros.h>
#include<geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Polygon.h>
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

// Текущая локальная карта
vector<int> localMap;
// Текущая открытая глобальная карта
vector<int> globalMap;

nav_msgs::OccupancyGrid globalMapMessage;

// Размер карты
int localMapSize = 0;
float mapResolution = 0.04;
//int globalMapSize = 20 / mapResolution;
int map_height = 20 / mapResolution;
int map_width = 30 / mapResolution;

//Текущее положение платформы
geometry_msgs::Point currentPosition;
// Угол поворота робота
float yawAngle;

bool isCameOdom = false;


void odometryCallback(const nav_msgs::Odometry data);
void connectLocalAndGlobalMaps();
void localMapCallback(nav_msgs::OccupancyGrid data);
void globalMapMessageInitParams();
void formGlobalMapMessage();

int main(int argc, char **argv){
  ros::init(argc, argv, "global_map_node");

  ros::NodeHandle m;
  ros::Subscriber local_map_sub = m.subscribe("/local_map", 8, localMapCallback);
  ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
  ros::Publisher global_map_pub = m.advertise<nav_msgs::OccupancyGrid>("/global_map", 2);

  bool isAllowProcess = true;
  globalMapMessageInitParams();
  ros::Rate rate(100);
  while(isAllowProcess && ros::ok()) {

    connectLocalAndGlobalMaps();
    // Формирование сообщения с картой
    formGlobalMapMessage();
    global_map_pub.publish(globalMapMessage);

    //cout << "Global map is coming" << endl;
    localMap.clear();

    rate.sleep();
    ros::spinOnce();
  }
}
void odometryCallback(const nav_msgs::Odometry data){
  // Получение угла поворота
  tf::Pose pose;
  tf::poseMsgToTF(data.pose.pose, pose);
  yawAngle = tf::getYaw(pose.getRotation());

  // Координата смещения лазера относительно центра платформы
  float laserOffsetX = 0.24;
  float laserOffsetY = 0;

  // Составляющая поворота
  float laserRotationX = laserOffsetX * cos(yawAngle) - laserOffsetY * sin(yawAngle);
  float laserRotationY = laserOffsetX * sin(yawAngle) + laserOffsetY * cos(yawAngle);

  // Окончательные начальные координаты
  currentPosition.x = data.pose.pose.position.x + laserRotationX;
  currentPosition.y = data.pose.pose.position.y + laserRotationY;

  cout << currentPosition.x << " " << currentPosition.y << " " << yawAngle << endl;

  isCameOdom = true;
}
void localMapCallback( nav_msgs::OccupancyGrid data){
  localMapSize = data.info.height;

  localMap.resize(data.info.height * data.info.width, 50);

  for(int i = 0; i < localMapSize * localMapSize; i++){
    localMap[i] = data.data[i];
  }
}

void connectLocalAndGlobalMaps(){
  if(isCameOdom){
    float k = 0.95;
    for(int i = 0; i < localMapSize; i++){
      for(int j = 0; j < localMapSize; j++){
        if(i < map_height && j < map_width){

          int rotationX = (i - localMapSize/2) * cos(yawAngle) - (j - localMapSize/2) * sin(yawAngle);
          int rotationY = (i - localMapSize/2) * sin(yawAngle) + (j - localMapSize/2) * cos(yawAngle);

          int x = map_width / 2 + currentPosition.x/mapResolution + (rotationX);
          int y = map_height / 2 + currentPosition.y/mapResolution + (rotationY);

          int value = globalMap[map_width * y + x] * k
              + localMap[localMapSize * j + i] * (1 - k);

          if(localMap[localMapSize * j + i] == 50){
            continue;
          }
          //        if(localMap[localMapSize * j + i] == 100 || globalMap[globalMapSize * y + x] == 100) {
          //          value = 100;
          //        }
          globalMap[map_width * y + x] = value;
        }
      }
    }
  }
}
void globalMapMessageInitParams(){

  globalMapMessage.info.height = map_height;
  globalMapMessage.info.width = map_width;
  globalMapMessage.info.resolution = mapResolution;
  globalMapMessage.info.origin.position.x = -map_width * mapResolution/2;
  globalMapMessage.info.origin.position.y = -map_height * mapResolution/2;
  globalMapMessage.header.frame_id = "/odom";
  globalMapMessage.header.stamp = ros::Time::now();
  globalMapMessage.data.resize(globalMapMessage.info.height * globalMapMessage.info.width);

  for(int i = 0; i < globalMapMessage.info.height * globalMapMessage.info.width; i++){
    globalMap.push_back(50);
  }
}

void formGlobalMapMessage(){
  for(int i = 0; i < globalMapMessage.info.height * globalMapMessage.info.width; i++){
    globalMapMessage.data[i] = globalMap[i];
  }
}
