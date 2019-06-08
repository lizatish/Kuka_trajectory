#include "rrt.h"

RRT::RRT(){
}

RRT::~RRT(){
  // Координаты старта и финиша
  delete start;
  delete end;
}
nav_msgs::Path RRT::Planning(geometry_msgs::Pose2D s, geometry_msgs::Pose2D g,
                             const nav_msgs::OccupancyGrid& gMap , float curv,
                             float robot_height, float robot_width, int maxIter0)
{
  // Параметры карты
  globalMap = gMap;
  map_height = gMap.info.height;
  map_width = gMap.info.width;
  mapResolution = gMap.info.resolution;

  // Параметры поля
  minRand = 0;
  if(map_height > map_width){
    maxRand = map_height*mapResolution;
  }
  else{
    maxRand = map_width*mapResolution;
  }

  // Параметры робота
  ROBOT_HEIGHT = robot_height;
  ROBOT_WIDTH = robot_width;
  CURVATURE = curv;

  // Параметры рассчеты
  goalSampleRate = 1; // овтечает за длину до рандомного узла (м)
  maxIter = maxIter0;

  //***************************************************************

  start = new Node(s.x + map_width/2*mapResolution, s.y + map_height/2*mapResolution, s.theta);
  end = new Node(g.x + map_width/2*mapResolution, g.y + map_height/2*mapResolution, g.theta);

  nodeList.push_back(start);
  Node* rnd = new Node();
  Node* newNode = new Node();
  vector<int> nearInds;
  ros::Time start_time = ros::Time::now();
  for(int i = 0; i < maxIter; i++){

    rnd = getRandomPoint();
    int nind = getNearestListIndex(rnd);
    newNode = steer(rnd, nind);

    if (collisionCheck(newNode)){
      nearInds = find_near_nodes(newNode);
      newNode = choose_parent(newNode, nearInds);
      nodeList.push_back(newNode);
      rewire(nearInds);
    }
  }
  //  cout << ros::Time::now().toSec() - start_time.toSec() << endl;
  int lastIndex = get_best_last_index();
  nav_msgs::Path path = gen_final_course(lastIndex);
  path.header.frame_id = "/odom";
  path.header.stamp = ros::Time::now();

  delete rnd, newNode;
  return path;
}
Node* RRT::choose_parent(Node* newNode, vector<int> nearInds){
  if (nearInds.size() == 0){
    return newNode;
  }

  vector<float> dlist;
  dlist.reserve(30);
  Node* tNode = new Node();
  for(int i = 0; i < nearInds.size(); i++){

    tNode = steer(newNode, nearInds[i]);
    if (collisionCheck(tNode)){
      dlist.push_back(tNode->cost);
    }
    else{
      dlist.push_back(INFINITY);
    }
  }
  delete tNode;

  float mincost = *min_element(dlist.begin(), dlist.end());
  vector<float>::iterator it = find(dlist.begin(), dlist.end(), mincost);
  int minind = nearInds[distance(dlist.begin(), it)];

  if (mincost == INFINITY){
    return newNode;
  }

  newNode = steer(newNode, minind);
  return newNode;
}
float RRT::pi_2_pi(float angle){
  return fmod((angle + M_PI), (2 * M_PI) - M_PI);
}
Node* RRT::steer(Node* rnd, int nind){

  Node* nearestNode = nodeList[nind];

  DubinsPathPlanning* DP;
  DubinsPathPlanning::originPath path;
  path = DP->dubins_path_planning(
        nearestNode->x, nearestNode->y, nearestNode->yaw,
        rnd->x, rnd->y, rnd->yaw, CURVATURE);

  Node* newNode = new Node;
  if(path.yaw.size() > 0){
    newNode->yaw = path.yaw[path.yaw.size()-1];
    newNode->path_yaw = path.yaw;
  }
  if(path.x.size() > 0){
    newNode->x = path.x[path.x.size()-1];
    newNode->y = path.y[path.y.size()-1];

    newNode->path_x = path.x;
    newNode->path_y = path.y;
  }
  float nearNodeCost = nearestNode->cost;
  newNode->cost = nearNodeCost + path.cost;
  newNode->parent = nind;

  return newNode;
}
Node* RRT::getRandomPoint(){

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> random(0, 100);
  std::uniform_real_distribution<> random_xy(minRand, maxRand);
  std::uniform_real_distribution<> random_yaw(-M_PI, M_PI);

  geometry_msgs::Point rnd;
  if (random(gen) > goalSampleRate){
    rnd.x = random_xy(gen);
    rnd.y = random_xy(gen);
    rnd.z = random_yaw(gen);
  }
  else{
    rnd.x = end->x;
    rnd.y = end->y;
    rnd.z = end->yaw;
  }

  Node* node = new Node(rnd.x, rnd.y, rnd.z);
  return node;
}
float RRT::get_best_last_index(){

  const float YAWTH = 1 * M_PI / 180;
  const float XYTH = 0.5;

  vector<int> goalinds;
  Node* node = new Node();
  for (int k = 0; k < nodeList.size(); k++){
    node = nodeList[k];
    if(calc_dist_to_goal(node->x, node->y) <= XYTH){
      goalinds.push_back(k);
    }
  }

  vector<int> fgoalinds;
  for(int i = 0; i < goalinds.size(); i++){
    if (abs(nodeList[goalinds[i]]->yaw - end->yaw) <= YAWTH){
      fgoalinds.push_back(goalinds[i]);
    }
  }

  if (fgoalinds.size() == 0){
    return NAN;
  }

  vector<float> cost;
  for(int i = 0; i < fgoalinds.size(); i++){
    cost.push_back(nodeList[fgoalinds[i]]->cost);
  }
  float mincost = *min_element(cost.begin(), cost.end());

  for(int i = 0; i < fgoalinds.size(); i++){
    if (nodeList[fgoalinds[i]]->cost == mincost){
      return fgoalinds[i];
    }
  }
  return NAN;
}
nav_msgs::Path RRT::gen_final_course(int goalInd){
  nav_msgs::Path path;

  if(goalInd < -1){
    return path;
  }

  geometry_msgs::PoseStamped p;
  p.pose.position.x = end->x - map_width/2*mapResolution;
  p.pose.position.y = end->y - map_height/2*mapResolution;
  p.pose.orientation.z = end->yaw;
  path.poses.push_back(p);

  Node* node = new Node();
  while(nodeList[goalInd]->parent != -1){
    node = nodeList[goalInd];

    for(int i = node->path_x.size() - 1; i >= 0; i--){
      p.pose.position.x = (node->path_x[i] - map_width/2*mapResolution);
      p.pose.position.y = (node->path_y[i] - map_height/2*mapResolution);
      p.pose.orientation.z = node->path_yaw[i];
      path.poses.push_back(p);

    }
    goalInd = node->parent;
  }
  return path;
}

float RRT::calc_dist_to_goal(float x, float y){
  return sqrt(pow(x - end->x, 2) + pow(y - end->y, 2));
}

vector<int> RRT::find_near_nodes(Node* newNode){
  int nnode = nodeList.size();
  float r_2 = pow(50.0 * sqrt((log(nnode) / nnode)),2);

  vector<float> dlist;
  float k = 0;
  for(int i = 0; i < nodeList.size(); i++){
    k = pow(nodeList[i]->x - newNode->x, 2) +
        pow(nodeList[i]->y - newNode->y, 2) +
        pow(nodeList[i]->yaw - newNode->yaw, 2);

    dlist.push_back(k);
  }

  vector<int> nearinds;
  for(int i = 0; i < dlist.size(); i++){
    if(dlist[i] <= r_2){
      nearinds.push_back(i);
    }
  }
  return nearinds;
}

void RRT::rewire(vector<int> nearInds){

  int nnode = nodeList.size();
  Node* nearNode = new Node();
  Node* tNode = new Node();

  bool obstacleOK = false;
  bool imporveCost = false;
  for(int i = 0; i < nearInds.size(); i++){
    nearNode = nodeList[nearInds[i]];
    tNode = steer(nearNode, nnode - 1);

    obstacleOK = collisionCheck(tNode);
    imporveCost = nearNode->cost > tNode->cost;

    if (obstacleOK && imporveCost){
      nodeList[nearInds[i]] = tNode;
    }
  }
}

int RRT::getNearestListIndex(Node* rnd){
  vector<float> dlist;
  float n = 0;
  float min_value = FLT_MAX;
  int min_index = 0;
  for(int i = 0; i < nodeList.size(); i++){
    n = pow(nodeList[i]->x - rnd->x, 2) +
        pow(nodeList[i]->y - rnd->y, 2) +
        pow(nodeList[i]->yaw - rnd->yaw, 2);
    dlist.push_back(n);
  }

  min_value = *min_element(dlist.begin(), dlist.end());
  vector<float>::iterator it = find(dlist.begin(), dlist.end(), min_value);
  min_index = distance(dlist.begin(), it);

  return min_index;
}

//// Проверка на пересечение с препятствиями
bool RRT::collisionCheck(Node* node){
  int start_size_x = -ROBOT_HEIGHT/(2*mapResolution);
  int start_size_y = -ROBOT_WIDTH/(2*mapResolution);
  int finish_size_x = ROBOT_HEIGHT/(2*mapResolution);
  int finish_size_y = ROBOT_WIDTH/(2*mapResolution);

  for(int k = 0; k < node->path_x.size(); k++){

    int x_robot_center = int(node->path_x[k]/mapResolution - globalMap.info.origin.position.x / globalMap.info.resolution);
    int y_robot_center = int(node->path_y[k]/mapResolution  - globalMap.info.origin.position.y / globalMap.info.resolution);
    float robot_yaw = node->path_yaw[k];

    float sin_yaw = sin(robot_yaw);
    float cos_yaw = cos(robot_yaw);

    if(x_robot_center + start_size_x >= 0 && x_robot_center + finish_size_x < globalMap.info.width
       && y_robot_center + start_size_y >= 0 && y_robot_center + finish_size_y < globalMap.info.height)
      for(int i = start_size_x; i <= finish_size_x; i++){
        for(int j = start_size_y; j <= finish_size_y; j++){

          // Составляющая поворота
          int x_robot_size = x_robot_center + i * cos_yaw + j * sin_yaw;
          int y_robot_size = y_robot_center - i * sin_yaw + j * cos_yaw;

          if(int(globalMap.data[map_width * y_robot_size + x_robot_size]) > 75){
//            cout << (int)globalMap.data[map_width * y_robot_size + x_robot_size] << endl;
            return false;
          }
        }
      }
  }
  return true;
}
