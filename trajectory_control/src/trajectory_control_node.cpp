#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <signal.h>
#include <termios.h>
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

void odometryCallback(const nav_msgs::Odometry &data);
void publishCommandVelocities(const geometry_msgs::Twist &data);
void targetPathCallback(const nav_msgs::Path &data);
void goToNewCoordinates();

void mySigintHandler(int sig);
static int kfd = 0;
static struct termios cooked, raw;

//Текущее положение и угол платформы
geometry_msgs::Point currentPosition;
float yawAngle;

bool isCameOdom = false;
bool isCameTargetPath = false;

// Управляющее сообщение
geometry_msgs::Twist commandVelocities;
// Публикатор желаемой скорости базы
ros::Publisher kuka_movebase_publisher;
// Для хранения целевого путя
vector<geometry_msgs::Point> targetPath;

//// ИЗМЕНЯЕМЫЕ ПАРАМЕТРЫ
// Радиус поворота
const float CURVATURE = 0.5;
// Минимальне расстояние для достижения цели
float DIST_TO_TARGET_MIN = 0.2;
/////////////////////////////

int main(int argc, char **argv){
  // Инициализация ROS
  ros::init(argc, argv, "kuka_traffic_control");

  ros::NodeHandle m;
  ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
  ros::Subscriber target_path_sub = m.subscribe("/target_path", 8, targetPathCallback);

  kuka_movebase_publisher = m.advertise<geometry_msgs::Twist> ("kuk_keyboard_control/pose_commands", 32);

  //Функция при завершении работы
  signal(SIGINT, mySigintHandler);

  bool isAllowProcess = true;
  ros::Rate rate(100);
  while(isAllowProcess && ros::ok()) {
    if(isCameTargetPath && isCameOdom){
      goToNewCoordinates();
    }
    rate.sleep();
    ros::spinOnce();
  }
}
void targetPathCallback(const nav_msgs::Path &data){
  targetPath.clear();
  for(int i = 0; i < data.poses.size(); i++){
    geometry_msgs::PoseStamped point = data.poses.at(i);
    geometry_msgs::Point p;
    p.x = point.pose.position.x;
    p.y = point.pose.position.y;
    p.z = point.pose.position.z;
    targetPath.push_back(p);
  }

  isCameTargetPath = true;
  cout << "Target path changed, new size is " << targetPath.size() << endl;
}
void odometryCallback(const nav_msgs::Odometry &data){
  // Получение угла поворота
  tf::Pose pose;
  tf::poseMsgToTF(data.pose.pose, pose);
  yawAngle = tf::getYaw(pose.getRotation());

  // Окончательные начальные координаты
  currentPosition.x = data.pose.pose.position.x;
  currentPosition.y = data.pose.pose.position.y;

  isCameOdom = true;
}

void goToNewCoordinates(){
  geometry_msgs::Twist data;
  cout << "Tar path size " << targetPath.size() << endl;

  if(targetPath.size()){
    float distToTarget = sqrt(pow((currentPosition.y) - targetPath[0].y, 2)
        + pow((currentPosition.x) - targetPath[0].x, 2));

    cout << "x " << currentPosition.x << " y " << currentPosition.y  << " dist " << distToTarget << endl;
    cout << "Target x: " << targetPath[0].x << " y: " << targetPath[0].y << endl;

    // Если не достигнуто предельное расстояние до цели
    if(distToTarget > DIST_TO_TARGET_MIN){
      distToTarget = sqrt(pow((currentPosition.y) - targetPath[0].y, 2)
          + pow((currentPosition.x) - targetPath[0].x, 2));
      float targetAngle = targetPath[0].z;
      float angleDiff = yawAngle - targetAngle;
      if(angleDiff > M_PI)
        angleDiff -= 2 * M_PI;
      if(angleDiff < -M_PI)
        angleDiff += 2 * M_PI;
      //      cout << " curAng " << yawAngle << " tarAng " << targetAngle << " diff " << angleDiff << endl;

      data.linear.x = 0.6;
      if(abs(angleDiff) > 0.01){
        if(angleDiff > 0.01)
          data.angular.z = -0.6/CURVATURE;
        else
          data.angular.z = 0.6/CURVATURE;
      }
      else{
        data.angular.z = 0;
        data.linear.x = 1;
      }
      // Задание постоянной ходовой скорости
      publishCommandVelocities(data);
    }
    // Если расстояние достигнуто
    else{
      // Удаляем достигнутую точку, если она есть
      targetPath.erase(targetPath.begin());
    }
  }
  // Если вектор с целями пуст, то посылаем нулевые скорости
  else{
    data.angular.z = 0;
    data.linear.x = 0;
    publishCommandVelocities(data);
  }
}

void publishCommandVelocities(const geometry_msgs::Twist &data){

  //Формируем пакет из управления
  commandVelocities.linear.x = data.linear.x;
  commandVelocities.linear.y = data.linear.y;
  commandVelocities.angular.z = data.angular.z;

  //Публикуем
  kuka_movebase_publisher.publish(commandVelocities);
  cout<<"Publish command velocities " << endl;
}
//Функция вызова при выключении
void mySigintHandler(int sig) {

  ROS_INFO("KUK KEYBOARD CONTROL stopped");
  tcsetattr(kfd, TCSANOW, &cooked);

  commandVelocities.linear.x = 0;
  commandVelocities.linear.y = 0;
  commandVelocities.angular.z = 0;

  kuka_movebase_publisher.publish(commandVelocities);
  ros::spinOnce();
  exit(0);
}
