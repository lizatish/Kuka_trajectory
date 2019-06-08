#ifndef RRT_H
#define RRT_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <math.h>
#include <vector>
#include <iostream>

#include <random>

#include "node.h"
#include "dubinspathplanning.h"

using namespace std;

class RRT
{
private:

  // Все для карты
  nav_msgs::OccupancyGrid globalMap;
  float map_height;
  float map_width;
  float mapResolution;

  // Начало карты(0) и конец карты = map_height/mapResolution
  int minRand;
  int maxRand;

  // Координаты старта и финиша
  Node* start;
  Node* end;

  float goalSampleRate;
  // Количество итераций
  int maxIter;

  // Габариты робота
  float ROBOT_WIDTH;
  float ROBOT_HEIGHT;
  // Радиус поворота
  float CURVATURE;

  // Лист с открытыми узлами и путями к ним
  vector<Node*> nodeList;


  // Сгенерировать рандомный узел
  Node* getRandomPoint();
  // Получить ближайший индекс узла
  int getNearestListIndex(Node* rnd);
  // Проложить путь до рандомного узла
  Node* steer(Node* rnd, int nind);
  // Проверка на пересечение с препятствиями
  bool collisionCheck(Node* node);
  // Выбрать родителя
  Node* choose_parent(Node* newNode, vector<int> nearInds);
  // Переписать узел, если это возможно
  void rewire(vector<int> nearinds);
  // Найти ближайшие узлы
  vector<int> find_near_nodes(Node* newNode);

  // Сфлормировать окончательный курс
  nav_msgs::Path gen_final_course(int goalInd);
  // Получить лучший индекс
  float get_best_last_index();

  // Рассчитать расстояние до цели
  float calc_dist_to_goal(float x, float y);
  // Перевести метры в ячейки для планировщика
  float metrs2cells(float metrs);
  // Отбросить лишний угол и войти в диапазон [-PI;PI]
  float pi_2_pi(float angle);

public:
  RRT();
  ~RRT();

  // Начать планирование
  nav_msgs::Path Planning(geometry_msgs::Pose2D s, geometry_msgs::Pose2D g,
                                        const nav_msgs::OccupancyGrid& gMap, float curv,
                                        float robot_height, float robot_width ,int maxIter0 = 100);
};

#endif // RRT_H
