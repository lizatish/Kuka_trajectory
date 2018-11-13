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
void targetPointCallBack(const geometry_msgs::Point &data);
void targetPathCallback(const nav_msgs::Path &data);
void goToNewCoordinates();

void mySigintHandler(int sig);
static int kfd = 0;
static struct termios cooked, raw;

//Текущее положение платформы
geometry_msgs::Point currentPosition;
// Координаты текущей цели
geometry_msgs::Point targetPoint;

bool isCameTarget = false;
bool isCameOdom = false;
bool isCameTargetPath = false;

float yawAngle;

//Коэффициент для скорости
float speedCoeff = 1;

//Управляющее сообщение
geometry_msgs::Twist commandVelocities;
// Публикатор желаемой скорости базы
ros::Publisher kuka_movebase_publisher;

vector<geometry_msgs::Point> targetPath;
int main(int argc, char **argv){
  // Инициализация ROS
  ros::init(argc, argv, "kuka_traffic_control");

  ros::NodeHandle m;
  ros::Subscriber odom_sub = m.subscribe("/odom", 8, odometryCallback);
  ros::Subscriber target_point_sub = m.subscribe("/target_point", 8, targetPointCallBack);
  ros::Subscriber target_path_sub = m.subscribe("/path_rrt", 8, targetPathCallback);

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
  for(int i = 0; i < data.poses.size(); i++){
    geometry_msgs::PoseStamped point = data.poses.at(i);
    geometry_msgs::Point p;
    p.x = point.pose.position.x;
    p.y = point.pose.position.y;
    targetPath.push_back(p);
  }
  isCameTargetPath = true;
  cout << targetPath.size() << endl;
}
void targetPointCallBack(const geometry_msgs::Point &data){
  targetPoint = data;
  isCameTarget = true;
}

void odometryCallback(const nav_msgs::Odometry &data){
  // Получение угла поворота
  tf::Pose pose;
  tf::poseMsgToTF(data.pose.pose, pose);
  yawAngle = tf::getYaw(pose.getRotation());

  // Координата смещения лазера относительно центра платформы
  float laserOffsetX = 0.19;
  float laserOffsetY = 0;

  // Составляющая поворота
  float laserRotationX = laserOffsetX * cos(yawAngle) - laserOffsetY * sin(yawAngle);
  float laserRotationY = laserOffsetX * sin(yawAngle) + laserOffsetY * cos(yawAngle);

  // Окончательные начальные координаты
  currentPosition.x = data.pose.pose.position.x + laserRotationX;
  currentPosition.y = data.pose.pose.position.y + laserRotationY;
  //    cout << currentPosition.x << " " << currentPosition.y << " " << yawAngle << endl;

  isCameOdom = true;
}

void goToNewCoordinates(){
  geometry_msgs::Twist data;
  cout << "Tar path size " << targetPath.size() << endl;

  float distToTarget = sqrt(pow((currentPosition.y) - targetPath[0].y, 2)
                            + pow((currentPosition.x) - targetPath[0].x, 2));
  cout << "x " << currentPosition.x << " y " << currentPosition.y  << " dist " << distToTarget << endl;
  cout << "Target x: " << targetPath[0].x << " y: " << targetPath[0].y << endl;

   if(distToTarget > 0.1){
    distToTarget = sqrt(pow((currentPosition.y) - targetPath[0].y, 2)
                        + pow((currentPosition.x) - targetPath[0].x, 2));
    float targetAngle = atan2(targetPath[0].y - currentPosition.y, targetPath[0].x - currentPosition.x);

    float angleDiff = yawAngle - targetAngle;
    if(angleDiff > M_PI)
      angleDiff -= 2 * M_PI;
    if(angleDiff < -M_PI)
      angleDiff += 2 * M_PI;
    cout << " curAng " << yawAngle << " tarAng " << targetAngle << " diff " << angleDiff << endl;


    // П-регулятор для рулевой скорости
    float PKoeff = 3;
    data.linear.x = 1;
    if(abs(angleDiff) > 0.2){
      if(angleDiff > 0.2)
        data.angular.z = -1;
      else
        data.angular.z = 1;
    }
    else{
      data.angular.z = -angleDiff * PKoeff;
      data.linear.x = -angleDiff * PKoeff;
    }
    // Задание постоянной ходовой скорости
//    publishCommandVelocities(data);
  }
  else{
    targetPath.erase(targetPath.begin());
  }
  isCameTarget = false;
  isCameOdom = false;
}

void publishCommandVelocities(const geometry_msgs::Twist &data){

  //Формируем пакет из управления
  commandVelocities.linear.x = speedCoeff * data.linear.x;
  commandVelocities.linear.y = speedCoeff * data.linear.y;
  commandVelocities.angular.z = speedCoeff * data.angular.z;

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
