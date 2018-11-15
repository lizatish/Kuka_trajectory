///*           Описание модуля
// Модуль предназначен для обработки информации, поступаюшей
// с лазерного сакнера,
// и построения по этим данным локальной карты проходимости
///*

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>

#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

void laserScanCallback(const sensor_msgs::LaserScan data);
void setMap(vector<float> theta, vector<float> r);
void buildMapFromLaserScaner();
void localMapMessageInitParams();
void initLocalMap();

// Текущая локальная карта
vector<int> localMap;
// Сообщение с картой
nav_msgs::OccupancyGrid localMapMessage;
// Текущий скан
sensor_msgs::LaserScan current_scan;
// Ширина Куки
const float ROBOT_WIDTH_HALF = 0.6/2;
// Пришел ли скан
bool isActualScanData = false;

// Размер карты
// Разрешение карты
float mapResolution = 0.04;
int mapSize = 12 / mapResolution;

int main(int argc, char **argv){
  ros::init(argc, argv, "kuka_local_map_node");

  ros::NodeHandle k;
  ros::Subscriber ls_sub = k.subscribe("/scan", 8, laserScanCallback);
  ros::Publisher map_pub = k.advertise<nav_msgs::OccupancyGrid>("/local_map", 2);

  initLocalMap();
  bool isAllowProcess = true;
  ros::Rate rate = 100;
  while(isAllowProcess && ros::ok()) {
    localMapMessageInitParams();
    // Построние карты со сканера
    buildMapFromLaserScaner();

    // Формирование сообщения с картой
    for(uint i = 0; i < localMap.size(); i++){
      localMapMessage.data[i] = localMap[i];
    }

    map_pub.publish(localMapMessage);
    ros::spinOnce();
    rate.sleep();
    cout << "Local map is coming" << endl;
  }
}

// Получение данных с лазерного сканера
void laserScanCallback(const sensor_msgs::LaserScan data) {
  current_scan = data;
  isActualScanData = true;
}
void initLocalMap(){
  // Инициализация локальной карты нулевыми параметрами
  for(int i = 0; i < mapSize * mapSize; i++){
    localMap.push_back(50);
  }
}

// Сортировка данных с лазерного сканера
void buildMapFromLaserScaner(){
  if(isActualScanData) {
    vector<float> angles;
    vector<float> ranges;
    for(int id = 0; id < current_scan.ranges.size(); id++) {
      float theta = current_scan.angle_min + id * current_scan.angle_increment;
      float range = current_scan.ranges.at(id);
      angles.push_back(theta);
      ranges.push_back(range);
    }
    // Задать карту по данным со сканера
    setMap(angles,ranges);
  }
}

// Формирование карты с лазерного сканера
void setMap(vector<float> theta, vector<float> r){
  for(int i = 0; i < mapSize; i++)
    for(int j = 0; j < mapSize; j++){
      localMap[mapSize * j + i] = 50;
    }

  for(uint m = 0; m < r.size(); m++){
    //    if(r[m] >= 6){
    //      r[m] = 6;
    //      // Преобразование координат новой точки из ПСК в ДСК
    //      float x2 = r[m] * cos(theta[m])/mapResolution + mapSize/2;
    //      float y2 = r[m] * sin(theta[m])/mapResolution + mapSize/2;

    //      if(x2 >= 0 && x2 < mapSize && y2 >= 0 && y2 < mapSize) {
    //        for(float dist = 0; dist < r[m]; dist += mapResolution) {

    //          int x3 = dist * cos(theta[m])/mapResolution - ROBOT_WIDTH_HALF + mapSize/2;
    //          int y3 = dist * sin(theta[m])/mapResolution - ROBOT_WIDTH_HALF + mapSize/2;

    //          localMap[mapSize * y3 + x3] = 25;
    //        }
    //      }
    //    }
    if(r[m] < 6 && r[m] > ROBOT_WIDTH_HALF){

      // Преобразование координат новой точки из ПСК в ДСК
      int x2 = r[m] * cos(theta[m])/mapResolution + mapSize/2;
      int y2 = r[m] * sin(theta[m])/mapResolution + mapSize/2;

      if(x2 >= 0 && x2 < mapSize && y2 >= 0 && y2 < mapSize) {
        for(float dist = 0; dist < r[m]; dist += mapResolution) {

          int x3 = dist * cos(theta[m])/mapResolution - ROBOT_WIDTH_HALF + mapSize/2;
          int y3 = dist * sin(theta[m])/mapResolution - ROBOT_WIDTH_HALF + mapSize/2;

          localMap[mapSize * y3 + x3] = 0;
        }
        localMap[mapSize * y2 + x2] = 100;
        localMap[mapSize * (y2 + 1) + (x2 + 1)] = 100;
        localMap[mapSize * (y2 + 1) + x2] = 100;
        localMap[mapSize * (y2 + 1) + (x2 - 1)] = 100;
        localMap[mapSize * (y2 - 1) + (x2 + 1)] = 100;
        localMap[mapSize * (y2 - 1) + (x2 - 1)] = 100;
        localMap[mapSize * (y2 - 1) + x2] = 100;
        localMap[mapSize * y2 + (x2 - 1)] = 100;
        localMap[mapSize * y2 + (x2 + 1)] = 100;

      }
    }
  }
}
void localMapMessageInitParams(){
  localMapMessage.info.height = mapSize;
  localMapMessage.info.width = mapSize;
  localMapMessage.info.resolution = mapResolution;
  localMapMessage.info.origin.position.x = -mapSize * mapResolution/2;
  localMapMessage.info.origin.position.y = -mapSize * mapResolution/2;
  localMapMessage.header.frame_id = "/odom";
  localMapMessage.header.stamp = ros::Time::now();
  localMapMessage.data.resize(localMapMessage.info.height * localMapMessage.info.width);
}
