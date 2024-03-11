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
#include"PairHash.hpp"
#include <utility>
#include <algorithm> // std::shuffle
#include <random>    // std::default_random_engine
#include <chrono> 
// 管理地图，船，泊位，机器人的类
class PortManager
{

public:

struct Compare
   {
      bool operator()(const Object& a,const Object& b)const
      {
        double ratio1=a.money/a.dist;
        double ratio2=b.money/b.dist;
         return ratio1>ratio2;
      }
   };
   Map map;
   std::mutex m;
   int robot_num = 10, berth_num = 10, ship_num = 5;
   int boat_capacity;       // 每艘船的容积，是相等的
   int money, rt_money;     // 当前金钱总数,rt表示实时数据
   int frameId; // 当前帧序号
   bool readyOutput = 1;
   bool readyInput = 1;
   int thread_status = 1;
   int c;

   std::queue<Object> objectQueue;
   std::vector<Berth> berthVector;                                       // 管理泊位的数组,索引就是泊位id
   std::vector<Ship> shipVector, rt_shipVector;                          // 船的数组,索引就是船的id
   std::vector<Robot> robotVector, rt_robotVector;                       // 机器人的数组，索引就是机器人id
   std::unordered_map<std::pair<int, int>, Object, pair_hash> objectMap; // 存储物品的哈希表，键值是坐标。
   std::queue<Object> deleteQueue; //消失物品的队列

   // std::priority_queue<InstructQueue,std::vector<InstructQueue>,CompareInstructQueue> instruction;//指令队列，存储所有指令，注意，队列元素是队列，元素队列中才存储指令
   std::queue<std::string> robotInstruction;
   std::queue<std::string> shipInstruction;
   std::vector<std::set<Object, Compare>> path_of_move;
   std::vector<std::unordered_map<std::pair<int,int>,std::vector<MobileEquipment>,pair_hash>>pathofgood;
   std::vector<std::unordered_map<std::pair<int,int>,int,pair_hash> >distogood;
   
   void input();
   void run();         // 主函数，在main中调用这个函数
   void initData();    // 从标准输入初始化数据
   int readFrame();    // 从一帧中读取数据
   void outputFrame(); // 输出一帧..
   void cal_path_of_maxvalue(Berth b);//每个泊位对应的搬运队列
   bool isValid(int x, int y);
   std::vector<MobileEquipment> bfs( MobileEquipment start, MobileEquipment end);
   void all_path(std::queue<Object>&goods);
   void deleteObject();
   bool setdist(Object g,Berth b);
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
   path_of_move.resize(berth_num);
   pathofgood.resize(berth_num);
   distogood.resize(berth_num);
}
int PortManager::readFrame()
{

   scanf("%d%d", &frameId, &rt_money);
   int num;
   scanf("%d", &num);
   for (int i = 1; i <= num; i++)
   {
      int x, y, val;
      scanf("%d%d%d", &x, &y, &val);
      objectQueue.push(Object(x, y, val,frameId+1000));
      deleteQueue.push(Object(x, y, val,frameId+1000));
      objectMap.insert(std::make_pair(std::make_pair(x, y),Object(x, y, val,frameId+1000) ));
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
   all_path(objectQueue);

   return frameId;
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
void PortManager::deleteObject()
{
   while(!deleteQueue.empty()&&deleteQueue.front().disappearFrame<=frameId)
   {
      objectMap.erase(std::make_pair(deleteQueue.front().x,deleteQueue.front().y));
      deleteQueue.pop();
   }
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
bool PortManager::setdist(Object g,Berth b)
  { // 计算该货物到某泊位的距离并赋值
    // int distance=((this->x-b.x)*(this->x-b.x)+(this->y-b.y)*(this->y-b.y));
    int distance =distogood[b.id].at(std::make_pair(g.x, g.y));
    if (distance <= g.dist)
    {
      g.dist = distance;
      g.berthid = b.id;
      return true;
    }
    return false;
  }
void PortManager::cal_path_of_maxvalue(Berth b){//传入货物数组

      int max=0;
      
	for(auto good:objectMap){
      Object g=good.second;
      
		if(setdist(g,b)){
			path_of_move[g.berthid].erase(g);
		}
	}
	for(auto good:objectMap){
      Object g=good.second;
		if(g.dist>max){
			max=g.dist;
		}
	}
	//MaxIndexHeap heap(g);
	//max=heap.removemax();
	std::vector<Object> pathofvalue(objectMap.size());
	//MaxIndexHeap heap(g);//MaxIndexHeap定义比较方式
	struct CustomCompare
   {
      bool operator()(const Object& a,const Object& b)
      {
         return a.money>b.money;
      }
   };
   std::priority_queue<Object, std::vector<Object>, CustomCompare> heap;
   for(auto g:objectMap){
      Object good=g.second;
      heap.push(good);
   }
	

	for(int i=0;i<objectMap.size();i++){
		Object target=heap.top();
      heap.pop();
		pathofvalue.push_back(target);
	}
	for(int i=0;i<pathofvalue.size();i++){
		if(b.velocity<=2||b.time>=1200){
			if(pathofvalue[i].dist<=max*0.45&&pathofvalue[i].berthid==b.id){
			
				path_of_move[b.id].insert(pathofvalue[i]);//链表
			
			}
			continue;
		}
		if(pathofvalue[i].dist<=max*0.6&&pathofvalue[i].berthid==b.id){
			
			path_of_move[b.id].insert(pathofvalue[i]);//链表
			
		}
	}
   

   
  
}

bool PortManager::isValid(int x, int y) {
   std::vector<std::vector<Element>>&grid=map.grid;
    return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y].type != '#';
}

std::vector<MobileEquipment> PortManager::bfs( MobileEquipment start, MobileEquipment end) {
   std::vector<std::vector<Element>>&grid=map.grid;
    std::vector<std::vector<bool>> visited(grid.size(), std::vector<bool>(grid[0].size(), false));//访问标记
    std::queue<MobileEquipment> q;
    std::vector<MobileEquipment> path;//路径
    std::map<std::pair<int, int>, std::pair<int, int>> parent;//节点的上一个节点
    std::vector<std::pair<int, int>> directions{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};//上下左右
   
   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
   std::default_random_engine engine(seed);

    // 打乱directions数组
    std::shuffle(directions.begin(), directions.end(), engine);
    q.push(start);
    visited[start.x][start.y] = true;
    parent[{start.x, start.y}] = {-1, -1};

    while (!q.empty()) {
        MobileEquipment current = q.front();
        q.pop();

        if (current.x == end.x && current.y == end.y) {
            
            std::pair<int, int> cur = {end.x, end.y};
            while (cur.first != -1) {
                path.push_back(MobileEquipment(cur.first, cur.second));
                cur = parent[cur];
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            if (isValid(newX, newY) && !visited[newX][newY]) {
                q.push(MobileEquipment(newX, newY));
                visited[newX][newY] = true;
                parent[{newX, newY}] = {current.x, current.y};//其前一个节点只能是上一层（无障碍）或左右（有障碍）
            }
        }
    }

    return {}; 
}
void PortManager::all_path(std::queue<Object>&goods){
   for(int j=0;j<berthVector.size();j++){
      for(int i=0;i<goods.size();i++){
      Object good=goods.front();
      goods.pop();
		std::vector<MobileEquipment>path=bfs(MobileEquipment(berthVector[j].x,berthVector[j].y),MobileEquipment(good.x,good.y));
		pathofgood[j].insert(std::make_pair(std::make_pair(good.x,good.y),path));
		distogood[j].insert(std::make_pair(std::make_pair(good.x,good.y),path.size()-1));//
	}
   }

}