#pragma once
#include<queue>
#include"Map.hpp"
#include"Berth.hpp"
#include"Ship.hpp"
#include"Robot.hpp"
#include<vector>
//管理地图，船，泊位，机器人的类
class PortManager
{
   //存储指令的队列
   struct InstructQueue
   {
      std::queue<std::string> _queue;
      int priority;//值越小，该队列在instruction的顺序越靠前
      InstructQueue(int type):priority(type)
      {
  
      }
   };
   struct CompareInstructQueue
   {
      bool operator()(const InstructQueue& a,const InstructQueue& b)
      {
         return a.priority>b.priority;
      }
   };
   public:
   Map map;
   std::vector<Berth> berthVector;//管理泊位的数组,索引就是泊位id
   std::vector<Ship> shipVector;//船的数组,索引就是船的id
   std::vector<Robot> robotVector;//机器人的数组，索引就是机器人id
   std::priority_queue<InstructQueue,std::vector<InstructQueue>,CompareInstructQueue> instruction;//指令队列，存储所有指令，注意，队列元素是队列，元素队列中才存储指令
   
   std::vector<std::vector<int>> shipLocation;//记录坐标位置是否有船只，没有为0，有则int值为 船只id+1
   std::vector<std::vector<int>> robotLocation;//记录机器人，同上


   
   void run();//主函数，在main中调用这个函数
   void initData();//从标准输入初始化数据
   void readFrame();//从一帧中读取数据
   void outputFrame();//输出一帧..

   


};