#pragma once
#include "MobileEquipment.hpp"
#include <queue>
#include<deque>
#include <string>
#include <vector>
class Robot : public MobileEquipment
{
public:
  int id;
  std::pair<int,int> berth;
  int goods = 0;  // 是否携带货物。0未携带，1携带
  int status = 1; // 状态，0表示恢复状态，1表示正常状态
  int a_status=0;//主动避让状态，0表示，1表示正在主动避让
  std::pair<int,int> destination;
  std::deque<std::string> instructionQueue;
  std::deque<MobileEquipment> mInstructionQueue;
  Robot(int id,int x, int y, int goods, int status) : id(id),MobileEquipment(x, y), goods(goods), status(status)
  {
  }
  Robot()
  {
  }
  bool handlePath(const std::vector<MobileEquipment>& path, int id)
  {
    std::string sId = std::to_string(id);
      for (int i = 0; i < path.size() - 1; i++)
      {
        if (path[i + 1].x > path[i].x)
        {
          instructionQueue.push_back("move " + sId + " 3");
        }
        else if (path[i + 1].x < path[i].x)
        {
          instructionQueue.push_back("move " + sId + " 2");
        }
        else if (path[i + 1].y > path[i].y)
        {
          instructionQueue.push_back("move " + sId + " 0");
        }
        else if (path[i + 1].y < path[i].y)
        {
          instructionQueue.push_back("move " + sId + " 1");
        }
        for (int i = 1; i < path.size(); i++)
        {
          mInstructionQueue.push_back(path[i]);
        }
      }

      return true;

    
  }
};