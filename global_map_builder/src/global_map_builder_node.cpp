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

geometry_msgs::Polygon obstacleListMessage;

// Текущая локальная карта
vector<int> localMap;

// Текущая открытая глобальная карта
vector<int> globalMap;

nav_msgs::OccupancyGrid globalMapMessage;
// Ширина Куки
const float ROBOT_WIDTH_HALF = 0.45/2;

// Размер карты
int localMapSize = 0;
float mapResolution = 0.04;
int globalMapSize = 20 / mapResolution;

//Текущее положение платформы
geometry_msgs::Point currentPosition;
// Угол поворота робота
float yawAngle;

nav_msgs::OccupancyGrid localMap_metric;
bool isCameOdom = false;

void odometryCallback(const nav_msgs::Odometry data);
void connectLocalAndGlobalMaps();
void localMapCallback(nav_msgs::OccupancyGrid data);
void globalMapMessageInitParams();
void formGlobalMapMessage();
void drawCircleObstacles(nav_msgs::OccupancyGrid& map, float radius);
void formObstacleList();

int main(int argc, char **argv){
  ros::init(argc, argv, "kuka_global_map_node");

  ros::NodeHandle m;
  ros::Subscriber local_map_sub = m.subscribe("/local_map", 8, localMapCallback);
  ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
  ros::Publisher global_map_pub = m.advertise<nav_msgs::OccupancyGrid>("/global_map", 2);
  ros::Publisher obstacle_list_pub = m.advertise<geometry_msgs::Polygon>("/obstacle_list", 2);

  bool isAllowProcess = true;
  globalMapMessageInitParams();
  ros::Rate rate(100);
  while(isAllowProcess && ros::ok()) {

    connectLocalAndGlobalMaps();

    formObstacleList();

    cout << obstacleListMessage.points.size() << endl;

    obstacle_list_pub.publish(obstacleListMessage);

    // Формирование сообщения с картой
    formGlobalMapMessage();
    global_map_pub.publish(globalMapMessage);


    localMap.clear();
    obstacleListMessage.points.clear();

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
  float laserOffsetX = 0;
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

void formObstacleList(){
  for(int i = 0; i < globalMapSize; i++){
    for(int j = 0; j < globalMapSize; j++){
      if(globalMap[globalMapSize * j + i] >= 75){
        geometry_msgs::Point32 p;
        p.x = (i - globalMapSize/2)*mapResolution;
        p.y = (j - globalMapSize/2)*mapResolution;

        obstacleListMessage.points.push_back(p);
      }
    }
    //    cout << obstacleListMessage.points.size() << endl;
  }
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
    float k = 0.9;
    for(int i = 0; i < localMapSize; i++){
      for(int j = 0; j < localMapSize; j++){
        if(i < globalMapSize && j < globalMapSize){

          int rotationX = (i - localMapSize/2) * cos(yawAngle) - (j - localMapSize/2) * sin(yawAngle);
          int rotationY = (i - localMapSize/2) * sin(yawAngle) + (j - localMapSize/2) * cos(yawAngle);

          int x = globalMapSize / 2 + currentPosition.x/mapResolution + (rotationX);
          int y = globalMapSize / 2 + currentPosition.y/mapResolution + (rotationY);

          if(localMap[localMapSize * j + i] == 50){
            continue;
          }

          int value = globalMap[globalMapSize * int(y) + int(x)] * k
              + localMap[localMapSize * j + i] * (1 - k);

          globalMap[globalMapSize * y + x] = value;

        }
      }
    }
  }
}
void globalMapMessageInitParams(){

  globalMapMessage.info.height = globalMapSize;
  globalMapMessage.info.width = globalMapSize;
  globalMapMessage.info.resolution = mapResolution;
  globalMapMessage.info.origin.position.x = -globalMapSize * mapResolution/2;
  globalMapMessage.info.origin.position.y = -globalMapSize * mapResolution/2;
  globalMapMessage.header.frame_id = "/odom";
  globalMapMessage.header.stamp = ros::Time::now();
  globalMapMessage.data.resize(globalMapMessage.info.height * globalMapMessage.info.width);

  for(int i = 0; i < globalMapSize * globalMapSize; i++){
    globalMap.push_back(50);
  }
}

void formGlobalMapMessage(){
  for(int i = 0; i < globalMapSize*globalMapSize; i++){
    globalMapMessage.data[i] = globalMap[i];
  }

  drawCircleObstacles(globalMapMessage, ROBOT_WIDTH_HALF);
}

void drawCircleObstacles(nav_msgs::OccupancyGrid& map, float radius){

  // Проходим по всем координатам препятствий
  for(int i = 0; i < obstacleListMessage.points.size(); i++){
    geometry_msgs::Point32 p0 = obstacleListMessage.points.at(i);

    int x0 = p0.x/mapResolution + globalMapSize/2;
    int y0 = p0.y/mapResolution + globalMapSize/2;
    // Рисуем препятствие
    map.data[globalMapSize * (y0) + (x0)] = 100;

    // Отрисовка кругов
    for(int p = x0 - int(radius / mapResolution) - 1; p < x0 + int(radius/ mapResolution + 1); p++) {
      for(int q = y0 - int(radius / mapResolution - 1); q < y0 + int(radius / mapResolution + 1); q++) {

        float dist =  sqrt(pow((p - x0), 2) + pow((q - y0), 2))*mapResolution;
        if(dist > radius)
          continue;

        // Рисуем то, что будет кругом впоследствии
        map.data[globalMapSize * q + p] = 100;
      }
    }
  }
}
