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
// 管理地图，船，泊位，机器人的类
struct pair_hash
{
   template <class T1, class T2>
   std::size_t operator()(const std::pair<T1, T2> &pair) const
   {
      auto hash1 = std::hash<T1>{}(pair.first);
      auto hash2 = std::hash<T2>{}(pair.second);
      // 使用一个常数来混合hash1和hash2，这里的常数31是一个小质数，但可以选择其他值
      return hash1 * 31 + hash2;
   }
};
class PortManager
{

public:
   Map map;
   std::mutex m;
   int robot_num = 10, berth_num = 10, ship_num = 5;
   int boat_capacity;       // 每艘船的容积，是相等的
   int money, rt_money;     // 当前金钱总数,rt表示实时数据
   int frameId, rt_frameId; // 当前帧序号
   bool readyOutput = 1;
   bool readyInput = 1;
   int thread_status = 1;
   int c;

   std::queue<Object> objectQueue;
   std::vector<Berth> berthVector;                                       // 管理泊位的数组,索引就是泊位id
   std::vector<Ship> shipVector, rt_shipVector;                          // 船的数组,索引就是船的id
   std::vector<Robot> robotVector, rt_robotVector;                       // 机器人的数组，索引就是机器人id
   std::unordered_map<std::pair<int, int>, Object, pair_hash> objectMap; // 存储物品的哈希表，键值是坐标。

   // std::priority_queue<InstructQueue,std::vector<InstructQueue>,CompareInstructQueue> instruction;//指令队列，存储所有指令，注意，队列元素是队列，元素队列中才存储指令
   std::queue<std::string> robotInstruction;
   std::queue<std::string> shipInstruction;

   void input();
   void run();         // 主函数，在main中调用这个函数
   void initData();    // 从标准输入初始化数据
   int readFrame();    // 从一帧中读取数据
   void outputFrame(); // 输出一帧..
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
      int x, y, time, velocity;
      // std::cerr<<"test ";
      // sleep(3);
      scanf("%d%d%d%d", &x, &y, &time, &velocity);
      berthVector.push_back(Berth(id, x, y, time, velocity));
   }

   scanf("%d", &boat_capacity);
   char okk[100];
   scanf("%s", okk);
   printf("OK\n");
   fflush(stdout);
   shipVector.resize(ship_num);
   rt_shipVector.resize(ship_num);
   robotVector.resize(robot_num);
   rt_robotVector.resize(robot_num);
}
int PortManager::readFrame()
{

   scanf("%d%d", &rt_frameId, &rt_money);
   int num;
   scanf("%d", &num);
   for (int i = 1; i <= num; i++)
   {
      int x, y, val;
      scanf("%d%d%d", &x, &y, &val);
      //objectQueue.push(Object(x, y, val));
      objectMap.insert(std::make_pair(std::make_pair(x, y),Object(x, y, val) ));
   }
   for (int i = 0; i < robot_num; i++)
   {
      int sts;
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
   return rt_frameId;
}
void PortManager::outputFrame()
{
   if (readyOutput)
   {
      while(!robotInstruction.empty())
      {
         std::cout<<robotInstruction.front()<<std::endl;
         robotInstruction.pop();
      }
      while(!shipInstruction.empty())
      {
         std::cout<<shipInstruction.front()<<std::endl;
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
void PortManager::run()
{

   initData();
   for(int i=0;i<15000;i++)
   {
      readFrame();


      outputFrame();
   }






   // std::thread input(&PortManager::input,this);
   // while (thread_status)
   // {
   //    //readyOutput = 0;
   //    while (readyInput)
   //    {

   //    }
   //    int i=10000;
   //    while(i--)
   //    {
   //     //std::cerr<<"test "<<std::endl;
   //     c=i+rand()%4;
   //    }

   //    //m.lock();
   //    readyOutput = 1;
   //    for (int i = 0; i < robot_num; i++)
   //    {
   //    printf("move %d %d\n", i, rand() % 4);
   //    }
   //    readyOutput=0;
   //    puts("OK");
   //    fflush(stdout);
   //    readyInput = 1;
   //    //m.unlock();

   // }
}