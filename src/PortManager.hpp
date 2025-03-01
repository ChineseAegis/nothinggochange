#pragma once
#include <queue>
#include "Map.hpp"
#include "Berth.hpp"
#include "Ship.hpp"
#include "Robot.hpp"
#include "Object.hpp"
#include <vector>
#include <thread>
#include <bits/stdc++.h>
#include <unordered_map>
#include <mutex>
#include <unistd.h>
#include "PairHash.hpp"
#include <utility>
#include <set>
#include <algorithm> // std::shuffle
#include <random>    // std::default_random_engine
#include <chrono>
using namespace std;
// #include <execution>
// #include <utility> // for std::pair
//  管理地图，船，泊位，机器人的类
class PortManager
{

public:
   struct Compare
   {
      bool operator()(const Object &a, const Object &b) const
      {
         // if(std::abs(a.disappearFrame-b.disappearFrame)>900)
         // {
         //    return a.disappearFrame>b.disappearFrame;
         // }
         double ratio1 = static_cast<double>(a.money) / a.dist;
         // double ratio1 = a.money / a.dist;
         double ratio2 = static_cast<double>(b.money) / b.dist;
         if (std::abs(ratio1 - ratio2) < 0.1)
         {
            return a.disappearFrame > b.disappearFrame;
         }
         return ratio1 < ratio2;
      }
   };
   struct CustomCompare
   {
      bool operator()(const Object &a, const Object &b)
      {
         return a.money > b.money;
      }
   };

   struct BerthTimeCompare
   {
      bool operator()(const Berth &a, const Berth &b)
      {
         return a.id < b.id;
      }
   };
   // struct Robot_hash
   // {
   //    std::size_t operator()(const Robot &r) const
   //    {
   //       auto hash1 = std::hash<int>{}(r.mInstructionQueue.front().x);
   //       auto hash2 = std::hash<int>{}(r.mInstructionQueue.front().y);
   //       // 使用一个常数来混合hash1和hash2，这里的常数31是一个小质数，但可以选择其他值
   //       return hash1 * 31 + hash2;
   //    }
   // };
   Map map;
   std::mutex m;
   int robot_num = 10, berth_num = 10, ship_num = 5;
   int boat_capacity;   // 每艘船的容积，是相等的
   int money, rt_money; // 当前金钱总数,rt表示实时数据
   int frameId;         // 当前帧序号
   bool readyOutput = 1;
   bool readyInput = 1;
   int thread_status = 1;
   int c;
   bool flag1 = 0; // 判断每次泊位的组合是否发生变化 0 未发生变化，1 发生变化
   int stayTime = 97;

   std::set<pair<double, int>> percent_berth; // 第一个是价值，第二个是berthid
   std::vector<Object> objectQueue;
   std::vector<Berth> berthVector;                                       // 管理泊位的数组,索引就是泊位id
   std::vector<Ship> shipVector, rt_shipVector;                          // 船的数组,索引就是船的id
   std::vector<Robot> robotVector, rt_robotVector;                       // 机器人的数组，索引就是机器人id
   std::unordered_map<std::pair<int, int>, Object, pair_hash> objectMap; // 存储物品的哈希表，键值是坐标。
   std::queue<Object> deleteQueue;                                       // 消失物品的队列
   std::unordered_map<int, std::unordered_map<std::pair<int, int>, bool, pair_hash>> robotLocation;
   std::vector<std::vector<int>> robotBerthReach;
   std::vector<std::vector<int>> berthReach;
   std::queue<Ship *> shipQueue;

   // std::priority_queue<InstructQueue,std::vector<InstructQueue>,CompareInstructQueue> instruction;//指令队列，存储所有指令，注意，队列元素是队列，元素队列中才存储指令
   std::queue<std::string> robotInstruction; // 管理机器人指令的队列，不要往这个队列插入机器人的move指令
   std::queue<std::string> shipInstruction;  // 管理船指令的队列
   std::vector<std::set<Object, Compare>> path_of_move;
   std::vector<double> berth_value; //
   std::vector<std::unordered_map<std::pair<int, int>, std::vector<MobileEquipment>, pair_hash>> pathofgood;
   std::vector<std::unordered_map<std::pair<int, int>, int, pair_hash>> distogood;
   std::vector<vector<int>> berth_result; // 分组结果
   void input();
   void run();         // 主函数，在main中调用这个函数
   void initData();    // 从标准输入初始化数据
   int readFrame();    // 从一帧中读取数据
   void berth_fen();   // 给泊位分组
   void outputFrame(); // 输出一帧..
   void DeepCopy2DVector(const std::vector<std::vector<int>> &src, std::vector<std::vector<int>> &dest);
   void cal_path_of_maxvalue(Berth &b, std::vector<Object> &goods); // 每个泊位对应的搬运队列
   void cal_berth_value(std::vector<Object> &goods);
   bool isValid(int x, int y, int frameId);
   std::vector<MobileEquipment> bfs(MobileEquipment start, MobileEquipment end);
   void all_goods_path(std::vector<Object> &goods);
   void all_path();
   void deleteObject();
   bool setdist(Object &g, Berth &b);
   bool moveRobot(int id, std::pair<int, int> destination, bool changeDestination = true, std::vector<MobileEquipment> *path = nullptr);
   void robotGet(int id);  // 延迟指令，不是实时生效的，可能要等机器人move到目的地后才生效
   void robotPull(int id); // 延迟指令，不是实时生效的，可能要等机器人move到目的地后才生效
   void insert_robot_move_instructions();
   void checkNextStep();
   void active_avoidance(int i);
   void checkRobot();
   void checkShip();
   void init_berth_dist();
   void selectionSortDesc(std::vector<double> &vec, vector<vector<int>> &vecc);
   void initRobotAndShip();
   void distributeObject();
   void processBerth(size_t j);
   void initRobotBerthReach();
};
void PortManager::initData()
{
   // char ch[210][210];
   //     for(int i = 1; i <= 200; i ++)
   //      scanf("%s", ch[i] + 1);
   map.ReadFromStandardInput();

   for (int i = 0; i < berth_num; i++)
   {

      int id;
      scanf("%d", &id);
      // cerr<<id<<endl;
      int x, y, time, velocity;
      // std::cerr<<"test ";
      // sleep(3);
      scanf("%d%d%d%d", &x, &y, &time, &velocity);
      berthVector.push_back(Berth(id, x, y, time, velocity));
   }

   scanf("%d", &boat_capacity);
   char okk[100];
   scanf("%s", okk);

   berth_value.resize(berth_num, 0);
   shipVector.resize(ship_num);
   rt_shipVector.resize(ship_num);
   robotVector.resize(robot_num);
   rt_robotVector.resize(robot_num);
   path_of_move.resize(berth_num);
   pathofgood.resize(berth_num);
   distogood.resize(berth_num);
   robotBerthReach.resize(10);
   berthReach.resize(10);
   for (int i = 0; i < ship_num; i++)
   {
      shipVector[i].id = i;
   }
   for (auto &r : robotBerthReach)
   {
      r.resize(10);
   }
   for (auto &r : berthReach)
   {
      r.resize(10);
   }
   stayTime = boat_capacity;
   // all_path();
   // stayTime=boat_capacity;
   // init_berth_dist();
   printf("OK\n");
   fflush(stdout);
}
int PortManager::readFrame()
{

   scanf("%d%d", &frameId, &money);
   int num;
   scanf("%d", &num);
   for (int i = 1; i <= num; i++)
   {
      int x, y, val;
      scanf("%d%d%d", &x, &y, &val);
      objectQueue.push_back(Object(x, y, val, frameId + 1000));
      deleteQueue.push(Object(x, y, val, frameId + 1000));
      objectMap.insert(std::make_pair(std::make_pair(x, y), Object(x, y, val, frameId + 1000)));
   }

   for (int i = 0; i < robot_num; i++)
   {
      int sts;
      robotVector[i].id = i;
      scanf("%d%d%d%d", &robotVector[i].goods, &robotVector[i].x, &robotVector[i].y, &robotVector[i].status);
   }
   for (int i = 0; i < 5; i++)
   {
      scanf("%d%d\n", &shipVector[i].status, &shipVector[i].berthId);
      shipVector[i].x = berthVector[i].x;
      shipVector[i].x = berthVector[i].x;
   }

   // if (readyInput)
   // {
   //    while (!objectQueue.empty())
   //    {
   //       Object o = objectQueue.front();
   //       objectMap.insert(std::make_pair(std::make_pair(o.x, o.y), o));
   //       objectQueue.pop();
   //    }
   //    for (int i = 0; i < robot_num; i++)
   //    {
   //       robotVector[i] = rt_robotVector[i];
   //    }
   //    for (int i = 0; i < ship_num; i++)
   //    {
   //       shipVector[i] = rt_shipVector[i];
   //    }
   //    // m.lock();
   //    readyInput = 0;
   //    // m.unlock();
   // }
   char okk[100];
   scanf("%s", okk);
   deleteObject();
   if (objectQueue.size())
   {
      all_goods_path(objectQueue);
      distributeObject();
      cal_berth_value(objectQueue);
      // for (int i = 0; i < 10; i++)
      // {

      //    cal_path_of_maxvalue(berthVector[i], objectQueue);
      // }
      objectQueue.clear();
   }
   if (frameId == 1)
   {
      initRobotBerthReach();
      initRobotAndShip();
   }
   // if (frameId % 500 == 0)
   // {
   //    berth_fen();
   // }

   return frameId;
}
void PortManager::cal_berth_value(std::vector<Object> &goods)
{
   for (auto g : goods)
   {
      if (g.berthid >= 0)
         berth_value[g.berthid] += static_cast<double>(g.money) / g.dist;
   }
}
void PortManager::outputFrame()

{
   if (readyOutput)
   {
      while (!robotInstruction.empty())
      {
         std::cout << robotInstruction.front() << std::endl;
         robotInstruction.pop();
      }
      while (!shipInstruction.empty())
      {
         std::cout << shipInstruction.front() << std::endl;
         shipInstruction.pop();
      }
      puts("OK");
      fflush(stdout);
   }
}
void PortManager::input()
{
   thread_status = 1;
   for (int i = 0; i < 15000; i++)
   {

      // if (i != 0)
      // {
      //    outputFrame();
      //    readFrame();
      // }else
      // {
      //    readFrame();
      // }
      readFrame();
      std::cerr << "test" << std::endl;
      std::this_thread::sleep_for(std::chrono::microseconds(14500));
      outputFrame();
   }
   // outputFrame();
   thread_status = 0;
}
void PortManager::deleteObject()
{
   while (!deleteQueue.empty() && deleteQueue.front().disappearFrame <= frameId)
   {
      // 这段是当删除物品时，把泊位附带的价值删去
      std::pair<int, int> temp_obj = std::make_pair(deleteQueue.front().x, deleteQueue.front().y);
      int t_money = objectMap[temp_obj].money;
      int t_dist = objectMap[temp_obj].dist;
      int t_id = objectMap[temp_obj].berthid;
      berth_value[t_id] -= static_cast<double>(t_money) / t_dist;

      //
      objectMap.erase(std::make_pair(deleteQueue.front().x, deleteQueue.front().y));
      deleteQueue.pop();
   }
   // robotLocation.erase(frameId);
}
void PortManager::run()
{
   initData();
   for (int i = 0; i < 15000; i++)
   {
      readFrame();
      checkRobot();
      checkShip();
      insert_robot_move_instructions();
      outputFrame();
   }
}
bool PortManager::setdist(Object &g, Berth &b)
{ // 计算该货物到某泊位的距离并赋值
   // int distance = ((g.x - b.x) * (g.x - b.x) + (g.y - b.y) * (g.y - b.y));
   int distance;
   if (distogood[b.id].find(std::make_pair(g.x, g.y)) != distogood[b.id].end())
   {
      distance = distogood[b.id].at(std::make_pair(g.x, g.y));
   }
   else
   {
      distance = std::numeric_limits<int>::max();
   }

   // int distance=10;
   if (distance < g.dist)
   {
      g.dist = distance;
      g.berthid = b.id;
      return true;
   }
   return false;
}
void PortManager::cal_path_of_maxvalue(Berth &b, std::vector<Object> &goods)
{ // 传入货物数组
   // path_of_move[b.id].clear();
   int max = 0;
   //
   for (int i = 0; i < goods.size(); i++)
   {
      Object &g = goods[i];
      int id = g.berthid;

      if (setdist(g, b) && id >= 0 && id != -1)
      {
         path_of_move[id].erase(g);
      }
   }

   // for (int i = 0; i < goods.size(); i++)
   // {
   //    Object &g = goods[i];
   //    if (g.dist > max)
   //    {
   //       max = g.dist;
   //    }
   // }
   // MaxIndexHeap heap(g);
   // max=heap.removemax();
   // std::vector<Object> pathofvalue;
   // MaxIndexHeap heap(g);//MaxIndexHeap定义比较方式

   // std::priority_queue<Object, std::vector<Object>, CustomCompare> heap;
   //  for (int i = 0; i < goods.size(); i++)
   //  {
   //     if (goods[i].berthid == b.id)
   //     {
   //        Object good = goods[i];
   //        heap.push(good);
   //     }
   //  }

   // while (!heap.empty())
   // {
   //    Object target = heap.top();
   //    heap.pop();
   //    // pathofvalue.push_back(target);
   //    pathofvalue.push_back(target);
   // }
   for (int i = 0; i < goods.size(); i++)
   {
      if (goods[i].berthid == b.id)
      {
         // std::cerr<<"test ";
         path_of_move[b.id].insert(goods[i]); // 链表
      }
   }
}

bool PortManager::isValid(int x, int y, int frameId)
{
   std::vector<std::vector<Element>> &grid = map.grid;
   // if(robotLocation[frameId].find(std::make_pair(x,y))!=robotLocation[frameId].end())
   // {
   //   std::cerr<<"test"<<std::endl;
   // }
   bool isRobot = 0;
   // for (int i = 0; i < 3; i++)
   // {
   //    for (int j = 0; j < 2; j++)
   //    {
   //       if (robotLocation[frameId - i].find(std::make_pair(x + j, y)) != robotLocation[frameId - i].end() && robotLocation[frameId - i].find(std::make_pair(x - j, y)) != robotLocation[frameId - i].end() &&
   //           robotLocation[frameId - i].find(std::make_pair(x, y + j)) != robotLocation[frameId - i].end() && robotLocation[frameId - i].find(std::make_pair(x + j, y + j)) != robotLocation[frameId - i].end() &&
   //           robotLocation[frameId - i].find(std::make_pair(x - j, y + j)) != robotLocation[frameId - i].end())
   //       {
   //          isRobot = 1;
   //       }
   //    }
   // }
   // for (int i = 0; i < 3; i++)
   // {
   //    for (int j = 0; j < 2; j++)
   //    {
   //       if (robotLocation[frameId - i].find(std::make_pair(x + j, y)) != robotLocation[frameId - i].end() && robotLocation[frameId - i].find(std::make_pair(x - j, y)) != robotLocation[frameId - i].end() &&
   //           robotLocation[frameId - i].find(std::make_pair(x, y + j)) != robotLocation[frameId - i].end() && robotLocation[frameId - i].find(std::make_pair(x + j, y + j)) != robotLocation[frameId - i].end() &&
   //           robotLocation[frameId - i].find(std::make_pair(x - j, y + j)) != robotLocation[frameId - i].end())
   //       {
   //          isRobot = 1;
   //       }
   //    }
   // }

   return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y].type != '#' && grid[x][y].type != '*' && !isRobot;
}

std::vector<MobileEquipment> PortManager::bfs(MobileEquipment start, MobileEquipment end)
{
   std::vector<std::vector<Element>> &grid = map.grid;
   // std::cerr << "test1 "<<endl;
   // std::cerr<<start.x<<" "<<start.y<<endl;
   // std::cerr<<end.x<<" "<<end.y<<endl;
   // std::cerr<<grid.size()<<endl;
   std::vector<std::vector<bool>> visited(200, std::vector<bool>(200, false)); // 访问标记
   // std::cerr << "test2 "<<endl;
   std::queue<MobileEquipment> q;
   std::vector<MobileEquipment> path;                                              // 路径
   std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> parent; // 节点的上一个节点
   std::vector<std::pair<int, int>> directions{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // 上下左右
   bool isreturn = true;

   MobileEquipment unvailed(-1, -1);
   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
   std::default_random_engine engine(seed);

   // 打乱directions数组
   std::shuffle(directions.begin(), directions.end(), engine);
   q.push(start);
   visited[start.x][start.y] = true;
   parent[{start.x, start.y}] = {-1, -1};
   int counter = frameId;
   while (!q.empty())
   {
      MobileEquipment current = q.front();
      q.pop();
      // if (isreturn && unvailed.x != -1 && unvailed.y != -1)
      // {
      //    // std::cerr<<"test"<<std::endl;
      //    std::pair<int, int> parent_of_current = parent.at(std::make_pair(current.x, current.y));
      //    if (abs(unvailed.x - parent_of_current.first) > 0)
      //       counter -= abs(unvailed.x - parent_of_current.first);
      //    if (abs(unvailed.y - parent_of_current.second) > 0)
      //       counter -= abs(unvailed.y - parent_of_current.second);
      //    counter--;
      // }
      if (current.x == end.x && current.y == end.y)
      {

         std::pair<int, int> cur = {end.x, end.y};
         while (cur.first != -1)
         {
            path.push_back(MobileEquipment(cur.first, cur.second));
            cur = parent[cur];
         }
         // reverse(path.begin(), path.end());
         return path;
      }
      // counter++;
      // isreturn = true;
      for (auto &dir : directions)
      {

         int newX = current.x + dir.first;
         int newY = current.y + dir.second;
         if (isValid(newX, newY, counter) && !visited[newX][newY])
         {
            // isreturn = false;
            q.push(MobileEquipment(newX, newY));
            visited[newX][newY] = true;
            parent[{newX, newY}] = {current.x, current.y}; // 其前一个节点只能是上一层（无障碍）或左右（有障碍）
         }
      }
      // if (isreturn)
      // {
      //    unvailed = current;
      // }
   }
   return {};
}
void PortManager::all_goods_path(std::vector<Object> &goods)
{
   // for (int j = 0; j < berthVector.size(); j++)
   // {
   //    for (int i = 0; i < goods.size(); i++)
   //    {
   //       Object good = goods[i];
   //       std::vector<MobileEquipment> path = bfs(MobileEquipment(berthVector[j].x, berthVector[j].y), MobileEquipment(good.x, good.y));
   //       if(path.size())
   //       {
   //       //pathofgood[j].insert(std::make_pair(std::make_pair(good.x, good.y), path));
   //       distogood[j].insert(std::make_pair(std::make_pair(good.x, good.y), path.size() - 1)); //
   //       }
   //    }
   // }
   // std::vector<std::mutex> locks(distogood.size()); // 为每个distogood元素提供一个锁

   // std::vector<std::mutex> locks(distogood.size()); // 为每个distogood元素提供一个锁
   std::vector<std::thread> outerThreads;

   for (size_t j = 0; j < berthVector.size(); ++j)
   {

      outerThreads.push_back(std::thread([&, j]
                                         {
        std::vector<std::thread> innerThreads;

        size_t batchSize = 3; // 每个线程处理的goods数量
        for (size_t i = 0; i < goods.size(); i += batchSize) {
            innerThreads.push_back(std::thread([&, j, i] {
                // 处理当前批次的每个goods元素
                for (size_t k = i; k < i + batchSize && k < goods.size(); ++k) {
                    auto path = bfs(MobileEquipment(berthVector[j].x, berthVector[j].y), MobileEquipment(goods[k].x, goods[k].y));
                    if (!path.empty()) {
                        m.lock();
                        distogood[j].insert(std::make_pair(std::make_pair(goods[k].x, goods[k].y), path.size() - 1));
                        m.unlock();
                    }
                }
            }));
        }

        // 等待内层所有线程完成
        for (auto& innerThread : innerThreads) {
            if (innerThread.joinable()) {
                innerThread.join();
            }
        } }));
   }

   // 等待外层所有线程完成
   for (auto &outerThread : outerThreads)
   {
      if (outerThread.joinable())
      {
         outerThread.join();
      }
   }
}
void PortManager::processBerth(size_t j)
{
   for (size_t i = 0; i < objectQueue.size(); ++i)
   {
      // 直接在这里执行BFS可能不是最高效的，并行化应该在有大量计算时才会带来好处
      // 如果BFS很快，那么创建和管理线程的开销可能会超过其带来的好处
      auto path = bfs(MobileEquipment(berthVector[j].x, berthVector[j].y), MobileEquipment(objectQueue[i].x, objectQueue[i].y));
      if (!path.empty())
      {
         // std::lock_guard<std::mutex> lock(locks[j]);
         m.lock();
         distogood[j].insert(std::make_pair(std::make_pair(objectQueue[i].x, objectQueue[i].y), path.size() - 1));
         m.unlock();
      }
   }
}
void PortManager::all_path()
{
   // for (int j = 0; j < berthVector.size(); j++)
   // {
   //    for (int x = 0; x < map.grid.size(); x++)
   //    {
   //       for (int y = 0; y < map.grid[j].size(); y++)
   //       {
   //          std::vector<MobileEquipment> path = bfs(MobileEquipment(berthVector[j].x, berthVector[j].y), MobileEquipment(x, y));
   //          //pathofgood[j].insert(std::make_pair(std::make_pair(x, y), path));
   //          distogood[j].insert(std::make_pair(std::make_pair(x, y), path.size() - 1));
   //       }
   //    }
   // }
   std::vector<std::thread> outerThreads;

   for (int j = 0; j < berthVector.size(); ++j)
   {
      outerThreads.push_back(std::thread([&, j]()
                                         {
            std::vector<std::thread> innerThreads;

            for (int x = 0; x < map.grid.size(); x += 100) {
                innerThreads.push_back(std::thread([&, j, x]() {
                    for (int i = x; i < std::min(x + 100, static_cast<int>(map.grid.size())); i+=20) {
                        for (int y = 0; y < map.grid[j].size(); y+=20) {
                           bool isbreak=0;
                              for(int c=0;c<20;c++)
                              {
                                 for(int d=0;d<20;d++)
                                 {
                                    if(isValid(c+i,d+y,1))
                                    {
                                       std::vector<MobileEquipment> path = bfs(MobileEquipment(berthVector[j].x, berthVector[j].y), MobileEquipment(c+i, d+y));
                                       //std::cerr<<c+i<<" "<<d+y<<std::endl;
                                       double result=static_cast<double>(path.size())/(std::abs(c+i-berthVector[j].x)+std::abs(d+y-berthVector[j].y));
                                       //std::cerr<<"result "<<result<<std::endl;
                                                                   // 使用互斥锁保护共享资源的写入
                                       std::lock_guard<std::mutex> guard(m);
                                       distogood[j].insert(std::make_pair(std::make_pair(c+i, d+y), path.size() - 1));
                                       isbreak=1;
                                       break;
                                    }
                                 }
                                 if(isbreak)
                                 {
                                    break;
                                 }
                              }                            

                        }
                    }
                }));
            }

            // 等待所有内层线程完成
            for (auto& t : innerThreads) {
                t.join();
            } }));
   }

   // 等待所有外层线程完成
   for (auto &t : outerThreads)
   {
      t.join();
   }
}

bool PortManager::moveRobot(int id, std::pair<int, int> destination, bool changeDestination, std::vector<MobileEquipment> *path)
{
   // for (int i = 0; i < berth_num; i++)
   // {
   //    if (robotVector[id].x == berthVector[i].x && robotVector[id].y == berthVector[i].y)
   //    {
   //       std::vector<MobileEquipment> path = pathofgood[i].at(std::make_pair(destination.first, destination.second));
   //       robotVector[id].handlePath(path, id);
   //       robotVector[id].destination = destination;
   //       return;
   //    }
   // }
   if (path != nullptr)
   {
      if (path->size() == 0)
      {
         return false;
      }
      else
      {
         robotVector[id].handlePath(*path, id, robotLocation, frameId);
         if (changeDestination)
         {
            robotVector[id].destination = destination;
         }
         return true;
      }
   }
   else
   {

      std::vector<MobileEquipment> path1 = bfs(MobileEquipment(robotVector[id].x, robotVector[id].y), MobileEquipment(destination.first, destination.second));
      if (path1.size() == 0)
      {
         return false;
      }
      else
      {
         robotVector[id].handlePath(path1, id, robotLocation, frameId);
         if (changeDestination)
         {
            robotVector[id].destination = destination;
         }
         return true;
      }
   }
}
void PortManager::robotGet(int id)
{
   std::string sId = std::to_string(id);
   robotVector[id].get_pull_instructions.push_back("get " + sId);
}
void PortManager::robotPull(int id)
{
   std::string sId = std::to_string(id);
   robotVector[id].get_pull_instructions.push_back("pull " + sId);
}

void PortManager::insert_robot_move_instructions()
{
   std::unordered_map<std::pair<int, int>, Robot *, pair_hash> map;
   for (int i = 0; i < robot_num; i++)
   {

      map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y), &robotVector[i]));
   }
   // std::cerr << "test " << std::endl;
   for (int i = 0; i < robot_num; i++)
   {
      if (!robotVector[i].mInstructionQueue.empty())
      {
         auto it = map.find(std::make_pair(robotVector[i].mInstructionQueue.front().x, robotVector[i].mInstructionQueue.front().y));
         int x = robotVector[i].mInstructionQueue.front().x;
         int y = robotVector[i].mInstructionQueue.front().y;
         // std::cerr << robotVector[i].mInstructionQueue.front().x << " " << robotVector[i].mInstructionQueue.front().y << std::endl;
         bool isRobot = 0;

         if (it == map.end())
         {
            map.insert(std::make_pair(std::make_pair(robotVector[i].mInstructionQueue.front().x, robotVector[i].mInstructionQueue.front().y), &robotVector[i]));
         }
         else if (it->second->a_status != i)
         {
            robotVector[i].instructionQueue.clear();
            robotVector[i].mInstructionQueue.clear();
            // robotVector[i].get_pull_instructions.clear();
            auto left = map.find(std::make_pair(robotVector[i].x - 1, robotVector[i].y));
            auto right = map.find(std::make_pair(robotVector[i].x + 1, robotVector[i].y));
            auto up = map.find(std::make_pair(robotVector[i].x, robotVector[i].y - 1));
            auto down = map.find(std::make_pair(robotVector[i].x, robotVector[i].y + 1));
            if (x > robotVector[i].x)
            {
               if (down == map.end() && moveRobot(i, std::make_pair(robotVector[i].x, robotVector[i].y + 1), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y + 1), &robotVector[i]));
               }
               else if (up == map.end() && moveRobot(i, std::make_pair(robotVector[i].x, robotVector[i].y - 1), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y - 1), &robotVector[i]));
               }
               else if (left == map.end() && moveRobot(i, std::make_pair(robotVector[i].x - 1, robotVector[i].y), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x - 1, robotVector[i].y), &robotVector[i]));
               }
            }
            else if (x < robotVector[i].x)
            {
               if (down == map.end() && moveRobot(i, std::make_pair(robotVector[i].x, robotVector[i].y + 1), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y + 1), &robotVector[i]));
               }
               else if (up == map.end() && moveRobot(i, std::make_pair(robotVector[i].x, robotVector[i].y - 1), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y - 1), &robotVector[i]));
               }
               else if (right == map.end() && moveRobot(i, std::make_pair(robotVector[i].x + 1, robotVector[i].y), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x + 1, robotVector[i].y), &robotVector[i]));
               }
            }
            else if (y > robotVector[i].y)
            {
               if (right == map.end() && moveRobot(i, std::make_pair(robotVector[i].x + 1, robotVector[i].y), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x + 1, robotVector[i].y), &robotVector[i]));
               }
               else if (left == map.end() && moveRobot(i, std::make_pair(robotVector[i].x - 1, robotVector[i].y), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x - 1, robotVector[i].y), &robotVector[i]));
               }
               else if (up == map.end() && moveRobot(i, std::make_pair(robotVector[i].x, robotVector[i].y - 1), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y - 1), &robotVector[i]));
               }
            }
            else if (y < robotVector[i].y)
            {
               if (right == map.end() && moveRobot(i, std::make_pair(robotVector[i].x + 1, robotVector[i].y), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x + 1, robotVector[i].y), &robotVector[i]));
               }
               else if (left == map.end() && moveRobot(i, std::make_pair(robotVector[i].x - 1, robotVector[i].y), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x - 1, robotVector[i].y), &robotVector[i]));
               }
               else if (down == map.end() && moveRobot(i, std::make_pair(robotVector[i].x, robotVector[i].y + 1), false))
               {
                  map.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
                  map.insert(std::make_pair(std::make_pair(robotVector[i].x, robotVector[i].y + 1), &robotVector[i]));
               }
            }
            // active_avoidance(i);
            robotVector[i].a_status = it->second->id;
         }
      }
   }
   // std::cerr << "end" << std::endl;
   for (int i = 0; i < robot_num; i++)
   {
      if (!robotVector[i].get_pull_instructions.empty())
      {
         robotInstruction.push(robotVector[i].get_pull_instructions.front());
         robotVector[i].get_pull_instructions.pop_front();
      }
      if (robotVector[i].status == 0 && robotVector[i].a_status >= 0)
      {
         robotVector[i].instructionQueue.clear();
         robotVector[i].mInstructionQueue.clear();
         robotVector[i].get_pull_instructions.clear();
         // active_avoidance(i);
         robotVector[i].a_status = robot_num;
      }
      else if (robotVector[i].status == 0 && robotVector[i].a_status >= 0)
      {
      }
      else
      {

         if (!robotVector[i].instructionQueue.empty() && !robotVector[i].mInstructionQueue.empty())
         {
            robotInstruction.push(robotVector[i].instructionQueue.front());
            robotVector[i].instructionQueue.pop_front();
            robotVector[i].mInstructionQueue.pop_front();
         }
      }
   }
}
void PortManager::active_avoidance(int i)
{
   // 使用系统时钟作为种子初始化随机数引擎
   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
   std::mt19937 generator(seed);

   // 定义一个在1到10之间均匀分布的随机整数
   std::uniform_int_distribution<int> distribution(1, 5);

   // 生成并打印两个随机数
   int random_number = distribution(generator);
   std::deque<std::string> dq;
   std::deque<MobileEquipment> mdq;
   int x = robotVector[i].x;
   int y = robotVector[i].y;
   for (int j = 0; j < random_number; j++)
   {
      int n = rand() % 4;
      dq.push_back("move " + std::to_string(i) + " " + std::to_string(n));
      if (n == 0)
      {
         mdq.push_back(MobileEquipment(x, y++));
      }
      else if (n == 1)
      {
         mdq.push_back(MobileEquipment(x, y--));
      }
      else if (n == 2)
      {
         mdq.push_back(MobileEquipment(x--, y));
      }
      else
      {
         mdq.push_back(MobileEquipment(x++, y));
      }
   }

   for (int j = 0; j < random_number; j++)
   {
      robotVector[i].instructionQueue.push_front(dq.back());
      robotVector[i].mInstructionQueue.push_front(mdq.back());
   }
   robotVector[i].a_status = 1;
}
void PortManager::checkRobot()
{
   double sum = 0;
   for (auto &value : berth_value)
   {
      sum += value;
   }
   // 分配船和泊位
   for (int i = 0; i < ship_num; i++)
   {
      if (shipVector[i].berthId >= 0 && shipVector[i].status == 1)
      {
         berthVector[shipVector[i].berthId].shipId = i;
         shipVector[i].time--;
      }

      // if(i==1)
      // {
      //    std::cerr<<shipVector[i].goods_num<<std::endl;
      // }
   }

   //
   for (int i = 0; i < berth_num; i++)
   {
      
      // if(!berthVector[i].vaild&&berthVector[i].isObject)
      // {
      //    cerr<<i<<":"<<berthVector[i].goods<<endl;
      // }
      int counter = berthVector[i].velocity;
      while (berthVector[i].goods && berthVector[i].shipId >= 0 && counter--)
      {
         berthVector[i].goods--;
         shipVector[berthVector[i].shipId].goods_num++;
      }
      // if (berthVector[i].f_shipId >= 0)
      // {
      //    berthVector[i].vaild = 1;
      //    berthVector[i].isObject = 1;
      // }

      if (!berthVector[i].vaild && berthVector[i].isObject)
      {
         if (!path_of_move[i].empty())
         {
            for (auto it = path_of_move[i].rbegin(); it != path_of_move[i].rend(); it++)
            {
               objectQueue.push_back(*it);
            }
         }
         path_of_move[i].clear();
         berthVector[i].isObject = 0;
      }
      // std::cerr<<berthVector[i].goods<<std::endl;
      // if (berthVector[i].vaild == 0 || 15000 - frameId - berthVector[i].velocity - 500 < 0)
      // {
      //    int distance = std::numeric_limits<int>::max();
      //    int berth = -1;
      //    int j = 0;
      //    for (auto b : berthReach[i])
      //    {
      //       if (b < distance && berthVector[i].vaild && b > 0 && j != i)
      //       {
      //          berth = j;
      //          distance = b;
      //       }
      //       j++;
      //    }
      //    if (berth >= 0&&path_of_move[berth].empty())
      //    {
      //       path_of_move[berth].insert(*path_of_move[i].rbegin());
      //       path_of_move[i].erase(std::prev(path_of_move[i].end()));

      //    }
      //    berthVector[i].vaild = 0;
      // }
   }
   // cerr<<"begin"<<endl;

   std::set<std::pair<double, int>> s;
   for (int n = 0; n < berth_num; n++)
   {
      s.insert(std::make_pair(berth_value[n], n));
   }

   double a_sum = 0;
   {
      int t = 0;
      for (auto &value : berth_value)
      {
         a_sum += (value / path_of_move[t].size());
      }
   }

   for (int i = 0; i < robot_num; i++)
   {
      robotVector[i].time--;
      //  std::cerr<<berth_value[i]<<endl;
      //   更改避障状态
      if (robotVector[i].berthId != robotVector[i].myberthId && !berthVector[robotVector[i].myberthId].vaild)
      {
         if (path_of_move[robotVector[i].berthId].empty() && !path_of_move[robotVector[i].myberthId].empty())
         {
            path_of_move[robotVector[i].berthId].insert(*path_of_move[robotVector[i].myberthId].rbegin());
            path_of_move[robotVector[i].myberthId].erase(std::prev(path_of_move[robotVector[i].myberthId].end()));
         }
      }
      if (!berthVector[robotVector[i].berthId].vaild)
      {

         {
            int distance = std::numeric_limits<int>::max();
            for (int j = 0; j < berth_num; j++)
            {
               // int distance1 = std::abs(robotVector[i].x - berthVector[j].x) + std::abs(robotVector[i].y - berthVector[j].y);
               int distance1 = berthReach[i][j];
               if (distance1 && berthVector[j].vaild && robotBerthReach[i][j] && j != i)
               {
                  if (distance1 < distance && berthVector[j].robot_num < 4)
                  {
                     berthVector[robotVector[i].berthId].robot_num--;
                     robotVector[i].berthId = j;
                     berthVector[robotVector[i].berthId].robot_num++;
                     distance = distance1;
                  }
               }
            }
         }
      }

      if (robotVector[i].a_status >= 0 && robotVector[i].instructionQueue.empty() && robotVector[i].get_pull_instructions.empty() && robotVector[i].mInstructionQueue.empty())
      {
         robotVector[i].a_status = -1;
         moveRobot(i, robotVector[i].destination);
      }
      if (robotVector[i].instructionQueue.empty() && robotVector[i].get_pull_instructions.empty() && robotVector[i].mInstructionQueue.empty())
      {
         auto it = objectMap.find(std::make_pair(robotVector[i].x, robotVector[i].y));
         if (robotVector[i].goods == 0)
         {

            if (it == objectMap.end())
            {

               if (robotVector[i].berthId != robotVector[i].myberthId && berthVector[robotVector[i].myberthId].vaild)
               {
                  if ((berth_value[robotVector[i].myberthId] / sum > 0.04 || berth_value[robotVector[i].berthId] / sum <= 0.1) && berthVector[robotVector[i].myberthId].vaild && berthVector[robotVector[i].myberthId].goods < boat_capacity)
                  {
                     berthVector[robotVector[i].berthId].robot_num--;
                     robotVector[i].berthId = robotVector[i].myberthId;
                     berthVector[robotVector[i].berthId].robot_num++;
                  }
               }

               if ((path_of_move[robotVector[i].berthId].empty() && frameId > 200) || !berthVector[robotVector[i].berthId].vaild)
               {
                  std::set<std::pair<double, int>> &s1 = s;
                  int id = s1.rbegin()->second;
                  auto it = s1.rbegin();
                  while ((!robotBerthReach[i][id] || !berthVector[id].vaild) && !s1.empty() && it != s1.rend())
                  {
                     it++;
                     if (it != s1.rend())
                        id = it->second;
                  }
                  // std::cerr << berth_value[id] / sum << endl;
                  if (robotBerthReach[i][id] && berth_value[id] / sum >= 0.2 && berthVector[id].vaild && berthVector[id].robot_num < (int)((berth_value[id] / sum) * 10))
                  {
                     berthVector[robotVector[i].berthId].robot_num--;
                     robotVector[i].berthId = id;
                     berthVector[robotVector[i].berthId].robot_num++;
                     robotVector[i].time = 300;
                  }
                  else
                  {
                     int distance = std::numeric_limits<int>::max();
                     for (int j = 0; j < berth_num; j++)
                     {
                        // int distance1 = std::abs(robotVector[i].x - berthVector[j].x) + std::abs(robotVector[i].y - berthVector[j].y);
                        int distance1 = berthReach[i][j];
                        if (distance1 && berthVector[j].vaild && robotBerthReach[i][j] && j != i && !path_of_move[j].empty())
                        {
                           if (distance1 < distance && berthVector[j].robot_num < 4)
                           {
                              berthVector[robotVector[i].berthId].robot_num--;
                              robotVector[i].berthId = j;
                              berthVector[robotVector[i].berthId].robot_num++;
                              distance = distance1;
                           }
                        }
                     }
                  }
               }

               if (berthVector[robotVector[i].berthId].goods > boat_capacity)
               {
                  int distance = std::numeric_limits<int>::max();
                  for (int j = 0; j < berth_num; j++)
                  {
                     // int distance1 = std::abs(robotVector[i].x - berthVector[j].x) + std::abs(robotVector[i].y - berthVector[j].y);
                     int distance1 = berthReach[i][j];
                     if (distance1 && berthVector[j].vaild && robotBerthReach[i][j] && j != i && !path_of_move[j].empty())
                     {
                        if (distance1 < distance && berthVector[j].robot_num < 4 && berthVector[robotVector[i].berthId].goods - berthVector[j].goods > 5)
                        {
                           berthVector[robotVector[i].berthId].robot_num--;
                           robotVector[i].berthId = j;
                           berthVector[robotVector[i].berthId].robot_num++;
                           distance = distance1;
                        }
                     }
                  }
               }

               if (!path_of_move[robotVector[i].berthId].empty())
               {
                  // std::cerr << "test ";
                  Object o = *path_of_move[robotVector[i].berthId].rbegin();
                  std::pair<int, int> temp_obj = std::make_pair(o.x, o.y);
                  if (objectMap.find(temp_obj) != objectMap.end())
                  {
                     int t_money = objectMap.at(temp_obj).money;
                     int t_dist = objectMap.at(temp_obj).dist;
                     int t_id = objectMap.at(temp_obj).berthid;
                     berth_value[t_id] -= static_cast<double>(t_money) / t_dist;
                  }
                  path_of_move[robotVector[i].berthId].erase(std::prev(path_of_move[robotVector[i].berthId].end()));
                  int distance = distogood[robotVector[i].berthId].at(std::make_pair(o.x, o.y));
                  // 判断是否在碰到前消失
                  // 当物品被移除时，删除泊位的价值
                  while (o.disappearFrame < frameId + distance)
                  {
                     if (!path_of_move[robotVector[i].berthId].empty())
                     {
                        Object o = *path_of_move[robotVector[i].berthId].rbegin();
                        // 当物品被移除时，删除泊位的价值
                        std::pair<int, int> temp_obj = std::make_pair(o.x, o.y);
                        if (objectMap.find(temp_obj) != objectMap.end())
                        {
                           int t_money = objectMap.at(temp_obj).money;
                           int t_dist = objectMap.at(temp_obj).dist;
                           int t_id = objectMap.at(temp_obj).berthid;
                           berth_value[t_id] -= static_cast<double>(t_money) / t_dist;
                        }

                        path_of_move[robotVector[i].berthId].erase(std::prev(path_of_move[robotVector[i].berthId].end()));
                        distance = distogood[robotVector[i].berthId].at(std::make_pair(o.x, o.y));
                     }
                     else
                     {
                        break;
                     }
                  }

                  if (o.disappearFrame > frameId + distance)
                  {

                     // std::cerr << o.x << " " << o.y << std::endl;
                     moveRobot(i, std::make_pair(o.x, o.y));
                  }
               }
            }
            else
            {

               robotGet(i);
               objectMap.erase(std::make_pair(robotVector[i].x, robotVector[i].y));
               Berth b = berthVector[robotVector[i].berthId];
               moveRobot(i, std::make_pair(b.x, b.y));
            }
         }
         else
         {
            if (robotVector[i].x - berthVector[robotVector[i].berthId].x <= 3 && robotVector[i].y - berthVector[robotVector[i].berthId].y <= 3 && robotVector[i].x - berthVector[robotVector[i].berthId].x >= 0 && robotVector[i].y - berthVector[robotVector[i].berthId].y >= 0)
            {
               robotPull(i);
               berthVector[robotVector[i].berthId].goods++;
            }
            else
            {
               Berth b = berthVector[robotVector[i].berthId];
               moveRobot(i, std::make_pair(b.x, b.y));
            }
         }
      }
      else if (robotVector[i].goods == 1 && robotVector[i].x - berthVector[robotVector[i].berthId].x <= 3 && robotVector[i].y - berthVector[robotVector[i].berthId].y <= 3 && robotVector[i].x - berthVector[robotVector[i].berthId].x >= 0 && robotVector[i].y - berthVector[robotVector[i].berthId].y >= 0)
      {
         robotPull(i);
         // shipVector[berthVector[robotVector[i].berthId].shipId].goods_num++;
         berthVector[robotVector[i].berthId].goods++;
         robotVector[i].instructionQueue.clear();
         robotVector[i].mInstructionQueue.clear();

         // Berth b = berthVector[robotVector[i].berthId];
         // moveRobot(i, std::make_pair(b.x, b.y));
      }
   }
}
void PortManager::checkShip()
{
   for (int i = 0; i < ship_num; i++)
   {

      if (((/*(frameId>10500&&shipVector[i].goback_counter==0)||*/ shipVector[i].goods_num >= boat_capacity) && shipVector[i].status == 1) || (15000 - frameId - berthVector[shipVector[i].berthId].time <= 2 && shipVector[i].status != 0))
      {
         // if(frameId>10500&&shipVector[i].goback_counter==0)
         //{
         shipVector[i].goback_counter = 1;
         //}
         // std::cerr<<"test"<<std::endl;
         // std::cerr<<"boat "<<i<<" is going to destination"<<std::endl;
         shipVector[i].goods_num = 0;
         shipInstruction.push("go " + std::to_string(i));
         berthVector[shipVector[i].berthId].shipId = -1;
         berthVector[shipVector[i].berthId].f_shipId = -1;
         shipVector[i].time = stayTime;
         if (15000 - frameId - 3 * berthVector[shipVector[i].berthId].time <= 10)
         {
            berthVector[shipVector[i].berthId].vaild = 0;
         }
         if (15000 - frameId - 3 * berthVector[(shipVector[i].berthId + 5) % 10].time <= 10)
         {
            berthVector[(shipVector[i].berthId + 5) % 10].vaild = 0;
         }
      }
      if (shipVector[i].time <= 0 && shipVector[i].status == 1 && (15000 - frameId - berthVector[(shipVector[i].berthId + 5) % 10].time - 500 > 30 && shipVector[i].status != 0) && berthVector[(shipVector[i].berthId + 5) % 10].vaild && boat_capacity - shipVector[i].goods_num > 5)
      {
         shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string((shipVector[i].berthId + 5) % 10));
         berthVector[shipVector[i].berthId].shipId = -1;
         berthVector[shipVector[i].berthId].f_shipId = -1;
         berthVector[(shipVector[i].berthId + 5) % 10].f_shipId = i;
         shipVector[i].time = stayTime;
         if (14000 - stayTime - frameId - berthVector[shipVector[i].berthId].time <= 10 || (15000 - frameId - 500 - berthVector[(shipVector[i].berthId + 5) % 10].time - 2 * berthVector[shipVector[i].berthId].time < 20 && boat_capacity - shipVector[i].goods_num - berthVector[(shipVector[i].berthId + 5) % 10].goods <= 5))
         {
            berthVector[shipVector[i].berthId].vaild = 0;
            // robotVector[i].berthId = (i + 5) % 10;
         }
      }
      else if ((15000 - frameId - berthVector[(shipVector[i].berthId + 5) % 10].time - 500 <= 10))
      {
         berthVector[(shipVector[i].berthId + 5) % 10].vaild = 0;
      }
   }
   for (int i = 0; i < ship_num; i++)
   {
      if (shipVector[i].berthId == -1 && shipVector[i].status != 0)
      {
         if (berthVector[shipVector[i].myBerthId].vaild && berthVector[(shipVector[i].myBerthId + 5) % 10].vaild && 15000 - frameId - 2 * berthVector[shipVector[i].myBerthId].time > 10 && 15000 - frameId - 2 * berthVector[(shipVector[i].myBerthId + 5) % 10].time > 10)
         {
            if (berthVector[shipVector[i].myBerthId].goods / berthVector[shipVector[i].myBerthId].velocity > berthVector[(shipVector[i].myBerthId + 5) % 10].goods / berthVector[(shipVector[i].myBerthId + 5) % 10].velocity)
            {
               shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string(shipVector[i].myBerthId));
               shipVector[i].time = stayTime;
               berthVector[shipVector[i].myBerthId].f_shipId = i;
            }
            else
            {
               shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string((shipVector[i].myBerthId + 5) % 10));
               shipVector[i].time = stayTime;
               berthVector[(shipVector[i].myBerthId + 5) % 10].f_shipId = i;
            }

            // std::cerr<<"boat "<<i<<" is going to berth "<<shipVector[i].myBerthId<<std::endl;
         }
         else if (berthVector[shipVector[i].myBerthId].vaild && 15000 - frameId - 2 * berthVector[shipVector[i].myBerthId].time > 10)
         {
            shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string(shipVector[i].myBerthId));
            shipVector[i].time = stayTime;
            berthVector[(shipVector[i].myBerthId + 5) % 10].vaild = 0;
            berthVector[shipVector[i].myBerthId].f_shipId = i;
         }
         else if (berthVector[(shipVector[i].myBerthId + 5) % 10].vaild && 15000 - frameId - 2 * berthVector[(shipVector[i].myBerthId + 5) % 10].time > 10)
         {
            shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string((shipVector[i].myBerthId + 5) % 10));
            shipVector[i].time = stayTime;
            berthVector[shipVector[i].myBerthId].vaild = 0;
            berthVector[(shipVector[i].myBerthId + 5) % 10].f_shipId = i;
         }
         else
         {
            berthVector[shipVector[i].myBerthId].vaild = 0;
            berthVector[(shipVector[i].myBerthId + 5) % 10].vaild = 0;
            int distance = std::numeric_limits<int>::max();
            int bid;
            for (int c = 0; c < berth_num; c++)
            {
               if (berthVector[i].time < distance)
               {
                  bid = i;
                  distance = berthVector[i].time;
               }
            }
            if (15000 - frameId - 2 * berthVector[bid].time > 15)
            {
               shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string(bid));
               berthVector[bid].f_shipId = i;
               berthVector[bid].vaild = 1;
               cerr << "ship go to berth:" << bid << endl;
               shipVector[i].time = stayTime;
            }
         }
      }
   }

   // for (int i = 0; i < ship_num; i++)
   // {
   //    if (shipVector[i].back_status == 1 && shipVector[i].status == 1 && shipVector[i].berthId == -1)
   //    {
   //       shipVector[i].back_status = 0;
   //       shipQueue.push(&shipVector[i]);
   //    }
   //    if ((shipVector[i].goods_num >= boat_capacity && shipVector[i].status == 1)|| 15000 - frameId - berthVector[shipVector[i].berthId].time <= 2)
   //    {
   //       shipVector[i].goods_num = 0;
   //       shipVector[i].back_status=1;
   //       shipInstruction.push("go " + std::to_string(i));
   //       berthVector[shipVector[i].berthId].shipId = -1;
   //       berthVector[shipVector[i].berthId].f_shipId = -1;
   //       if(15000-frameId-2*berthVector[shipVector[i].berthId].time<10)
   //       {
   //         berthVector[shipVector[i].berthId].vaild=0;
   //       }
   //    }
   // }
   // for (int i = 0; i < berth_num; i++)
   // {
   //    if (berthVector[i].goods > boat_capacity * 0.6&&berthVector[i].vaild)
   //    {
   //       if (!shipQueue.empty())
   //       {
   //          Ship* ship = shipQueue.front();
   //          shipQueue.pop();
   //          shipInstruction.push("ship " + std::to_string(ship->id) + " " + std::to_string(i));
   //          berthVector[i].shipId = ship->id;
   //          berthVector[i].f_shipId = ship->id;
   //       }
   //    }
   // }
}
void PortManager::initRobotAndShip()
{
   // for (int i = 0; i < berth_num; i++)
   // {
   //    cerr << berthVector[i].time << endl;
   // }
   std::vector<Berth> ordered_berth = berthVector;
   std::sort(ordered_berth.begin(), ordered_berth.end(), BerthTimeCompare());
   for (int i = 0; i < ship_num; i++)
   {
      int id;
      if (berthVector[i].time < berthVector[(i + 5) % 10].time)
      {
         id = i;
      }
      else
      {
         id = (i + 5) % 10;
      }
      shipVector[i].myBerthId = id;
      berthVector[id].shipId = i;
      berthVector[id].f_shipId = i;
      shipInstruction.push("ship " + std::to_string(i) + " " + std::to_string(id));
      // cerr<<"ship "<<i<<" going to berth "<<id<<endl;
      shipVector[i].time = stayTime;
      shipVector[i].status = 0;
   }

   // for (int i = 0; i < ship_num; i++)
   // {
   //    shipQueue.push(&shipVector[i]);
   // }
   int j = 0;
   for (int i = 0; i < robot_num; i++)
   {
      int n = 10;
      while (n--)
      {
         if (robotBerthReach[i][j])
         {
            robotVector[i].berthId = j;
            robotVector[i].myberthId = j;
            j = (j + 1) % robot_num;
            break;
         }
         j = (j + 1) % robot_num;
      }
   }
   // for(int i=0;i<robot_num;i++)
   // {
   //    cerr<<robotVector[i].berthId<<endl;
   // }

   // double sum = 0;
   // int i = 0;
   // for (auto &berth : berth_value)
   // {
   //    sum += berth;
   // }
   // for (auto &group : berth_result)
   // {
   //    double value;
   //    for (auto &berth : group)
   //    {
   //       value += berth_value[berth];
   //    }
   //    int num;
   //    if (value / sum > 1)
   //    {
   //       num = value / sum;
   //    }
   //    else
   //    {
   //       num = 1;
   //    }
   //    for (auto &berth : group)
   //    {
   //       if (num--)
   //       {
   //          int t = 10;
   //          while ((robotVector[i].berthId == -1 || !robotBerthReach[i][berth]) && t--)
   //          {
   //             i = (i + 1) % 10;
   //          }
   //          if (robotBerthReach[i][berth] && robotVector[i].berthId >= 0)
   //          {
   //             robotVector[i].berthId = berth;
   //             robotVector[i].berth = group;
   //             i = (i + 1) % 10;
   //          }
   //       }
   //       else
   //       {
   //          break;
   //       }
   //    }
   //    for (int c = 0; c < num; c = (c + 1) % group.size())
   //    {
   //       int t = 10;
   //       while (robotVector[i].berthId == -1 || !robotBerthReach[i][c] && t--)
   //       {
   //          i = (i + 1) % 10;
   //       }
   //       if (robotBerthReach[i][c] && robotVector[i].berthId >= 0)
   //       {
   //          robotVector[i].berthId = group[c];
   //          robotVector[i].berth = group;
   //          i = (i + 1) % 10;
   //       }
   //    }
   //    int n = 0;
   //    for (int j = 0; j < robot_num; j++)
   //    {
   //       if (robotVector[i].berthId == -1)
   //       {
   //          int size = berth_result[n].size();
   //          robotVector[i].berthId = berth_result[n][size - 1];
   //          n++;
   //       }
   //    }
   // }

   // robotVector[0].berthId = 0;
   // robotVector[1].berthId = 0;
   // robotVector[2].berthId = 2;
   // robotVector[3].berthId = 2;
   // robotVector[4].berthId = 4;
   // robotVector[5].berthId = 4;
   // robotVector[6].berthId = 6;
   // robotVector[7].berthId = 6;
   // robotVector[8].berthId = 8;
   // robotVector[9].berthId = 8;

   // shipInstruction.push("ship 0 0");
   // shipVector[0].myBerthId = 0;
   // berthVector[0].shipId = 0;
   // shipInstruction.push("ship 1 2");
   // shipVector[1].myBerthId = 2;
   // berthVector[2].shipId = 1;
   // shipInstruction.push("ship 2 4");
   // shipVector[2].myBerthId = 4;
   // berthVector[4].shipId = 2;
   // shipInstruction.push("ship 3 6");
   // shipVector[3].myBerthId = 6;
   // berthVector[6].shipId = 3;
   // shipInstruction.push("ship 4 8");
   // shipVector[4].myBerthId = 8;
   // berthVector[8].shipId = 4;
}

void PortManager::distributeObject()
{
   for (auto &object : objectQueue)
   {

      for (auto &berth : berthVector)
      {
         if (berth.vaild)
         {
            int distance = std::numeric_limits<int>::max();
            if (distogood[berth.id].find(std::make_pair(object.x, object.y)) != distogood[berth.id].end())
            {
               distance = distogood[berth.id].at(std::make_pair(object.x, object.y));
            }
            else
            {
               distance = std::numeric_limits<int>::max();
            }

            if (distance < object.dist)
            {
               object.berthid = berth.id;
               object.dist = distance;
            }
         }
      }
   }
   for (auto &object : objectQueue)
   {
      if (object.berthid >= 0)
         path_of_move[object.berthid].insert(object);
   }
}
void PortManager::init_berth_dist()
{
   std::vector<std::vector<int>> st(10, std::vector<int>(10, 0));
   for (auto &b : berthVector)
   {
      for (auto &bb : berthVector)
      {
         if (b.id == bb.id)
         {
         }
         else
         {
            std::vector<MobileEquipment> path;
            if (st[b.id][bb.id] == 0)
            {
               path = bfs(MobileEquipment(b.x, b.y), MobileEquipment(bb.x, bb.y));
               if (path.size() == 0)
               {
                  st[b.id][bb.id] = std::numeric_limits<int>::max();
                  ;
                  st[bb.id][b.id] = std::numeric_limits<int>::max();
                  ;
               }
               else
               {
                  st[b.id][bb.id] = path.size();
                  st[bb.id][b.id] = path.size();
               }
            }
            if (st[b.id][bb.id] > 0 && st[b.id][bb.id] <= 100)
            {
               b.berthid_clo.insert(std::make_pair(st[b.id][bb.id], bb.id));
               bb.berthid_clo.insert(std::make_pair(st[b.id][bb.id], b.id));
            }
         }
      }
   }
}
void PortManager::berth_fen()
{

   for (auto &b : berthVector)
   {
      b.flag = 0;
   }
   flag1 = 0; // 是否发生改变
   //
   vector<vector<int>> t_berth_result(10, std::vector<int>(10, -1));

   // 计算每个泊位的价值占比

   double sum_value = 0;
   for (auto &v : berth_value)
   {
      sum_value += v;
   }
   int i = 0;
   for (auto &v : berth_value)
   {
      berthVector[i].percent = v / sum_value;
      percent_berth.insert(make_pair(berthVector[i].percent, i));
      i++;
   }

   //
   // 找每个泊位的最近泊位
   // std::vector<std::vector<int>> st(10, std::vector<int>(10, 0));
   // for (auto &b : berthVector)
   // {
   //    for (auto &bb : berthVector)
   //    {
   //       if (b.id == bb.id)
   //          continue;
   //       else
   //       {
   //          std::vector<MobileEquipment> path;
   //          if (st[b.id][bb.id] == 0)
   //             path = bfs(MobileEquipment(b.x, b.y), MobileEquipment(bb.x, bb.y));
   //          if (path.size() > 0 && path.size() <= 100)
   //          {
   //             st[b.id][bb.id] = 1;
   //             st[bb.id][b.id] = 1;
   //             b.berthid_clo.insert(std::make_pair(path.size(), bb.id));
   //             bb.berthid_clo.insert(std::make_pair(path.size(), b.id));
   //          }
   //       }
   //    }
   // }

   ////
   // 泊位组合

   vector<double> val_fen(10); // 每个分组的百分比值

   int j = 0;        // 分组序列
   double t_val = 0; // 临时记录当前组的价值

   for (auto &b : percent_berth)
   {
      int k = 0;

      if (berthVector[b.second].flag == 1)
         continue;
      if (berthVector[b.second].berthid_clo.empty())
      {
         if (b.first < 0.05)
            continue;
      }
      t_val += b.first;

      t_berth_result[j][k++] = b.second;
      berthVector[b.second].flag = 1;
      // 如果本身已经大于0.1 直接不用与其他分组
      if (b.first > 0.1)
      {
         j++;
         continue;
      }

      while (1)
      {

         while (!berthVector[b.second].berthid_clo.empty())
         {
            int t_id = (*(berthVector[b.second].berthid_clo.begin())).second;
            if (berthVector[t_id].flag == 1)
               berthVector[b.second].berthid_clo.erase(berthVector[b.second].berthid_clo.begin());
            else
            {
               break;
            }
         }

         if (berthVector[b.second].berthid_clo.empty())
         {

            break;
         }
         int t_id = (*(berthVector[b.second].berthid_clo.begin())).second;
         berthVector[b.second].berthid_clo.erase(berthVector[b.second].berthid_clo.begin());
         t_berth_result[j][k++] = t_id;
         berthVector[t_id].flag = 1;
         t_val += berthVector[t_id].percent;

         if (t_val >= 0.1 && t_val != b.first)
         {
            if (!berthVector[b.second].berthid_clo.empty())
            {
               double sum = 0;
               vector<int> v;
               for (auto &x : berthVector[b.second].berthid_clo)
               {
                  int id = x.second;
                  if (berthVector[id].percent < 0.1)
                  {
                     sum += berthVector[id].percent;
                     v.push_back(id);
                  }
               }
               if (sum < 0.1)
               {
                  for (auto &h : v)
                  {
                     t_berth_result[j][k++] = h;

                     berthVector[h].flag = 1;
                     t_val += berthVector[h].percent;
                  }
               }
            }
         }
      }

      val_fen[j] = t_val;

      j++;
      t_val = 0;
   }

   // 第一步：根据 val_fen 对第一层数组进行排序
   selectionSortDesc(val_fen, t_berth_result);
   // 第二步：对每个第一层数组的第二层数组进行排序

   for (auto &vec : t_berth_result)
   {
      // 首先，移除所有值为-1的元素
      auto newEnd = std::remove_if(vec.begin(), vec.end(),
                                   [&](int id)
                                   { return id == -1 || berthVector[id].percent == -1; });
      vec.erase(newEnd, vec.end());

      // 然后，对剩余的元素进行降序排序
      std::sort(vec.begin(), vec.end(),
                [&](int a_id, int b_id)
                {
                   return berthVector[a_id].percent > berthVector[b_id].percent; // 降序
                });
   }

   // 判断是否没变
   if (berth_result != t_berth_result)
   {
      flag1 = 1;
   }

   swap(t_berth_result, berth_result);
}
// 排序

void PortManager::selectionSortDesc(std::vector<double> &vec, vector<vector<int>> &vecc)
{
   // vec.size() - 1，因为最后一个元素不需要再比较
   for (size_t i = 0; i < vec.size() - 1; ++i)
   {
      // 假设当前索引i为最大值的索引
      size_t maxIndex = i;
      // 从i + 1到vec.size() - 1中查找真正的最大值的索引
      for (size_t j = i + 1; j < vec.size(); ++j)
      {
         // 如果找到比当前“最大”更大的值，则更新maxIndex
         if (vec[j] > vec[maxIndex])
         {
            maxIndex = j;
         }
      }
      // 如果最大值的索引不是i，将其与i位置的值交换
      if (maxIndex != i)
      {
         std::swap(vec[i], vec[maxIndex]);
         swap(vecc[i], vecc[maxIndex]);
      }
   }
}
void PortManager::initRobotBerthReach()
{
   std::vector<std::thread> threads;
   for (int i = 0; i < 10; ++i)
   {
      // 创建线程，使用lambda函数处理每个机器人与所有泊位之间的关系
      threads.push_back(std::thread([&, i]()
                                    {
            for (int j = 0; j < 10; ++j) {
                if (bfs(MobileEquipment(robotVector[i].x, robotVector[i].y), MobileEquipment(berthVector[j].x, berthVector[j].y)).size()) {
                   m.lock();
                    robotBerthReach[i][j] = 1;
                    m.unlock();
                } else {
                  m.lock();
                    robotBerthReach[i][j] = 0;
                    m.unlock();
                }
            } }));
   }

   // 等待所有线程完成
   for (auto &thread : threads)
   {
      thread.join();
   }

   std::vector<std::thread> threads2;
   for (int i = 0; i < 10; ++i)
   {
      // 创建线程，使用lambda函数处理每个机器人与所有泊位之间的关系
      threads2.push_back(std::thread([&, i]()
                                     {
            for (int j = 0; j < 10; ++j) {
               std::vector<MobileEquipment> path=bfs(MobileEquipment(berthVector[i].x, berthVector[i].y), MobileEquipment(berthVector[j].x, berthVector[j].y));
                if (path.size()) {
                   m.lock();
                    berthReach[i][j] = path.size();
                    m.unlock();
                } else {
                  m.lock();
                    berthReach[i][j] = 0;
                    m.unlock();
                }
            } }));
   }

   // 等待所有线程完成
   for (auto &thread : threads2)
   {
      thread.join();
   }
}