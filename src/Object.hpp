#pragma once
#include "MobileEquipment.hpp"
#include <limits>
#include <iostream>
#include "Berth.hpp"
#include "PortManager.hpp"
class Object : public MobileEquipment
{
public:
  int money;
  int dist = std::numeric_limits<int>::max();
  int berthid;
  int disappearFrame;
  Object(int x, int y, int money,int disappearFrame) : MobileEquipment(x, y), money(money),disappearFrame(disappearFrame)
  {
  }
  Object()
  {
    
  }
  bool setdist(Berth b)
  { // 计算该货物到某泊位的距离并赋值
    // int distance=((this->x-b.x)*(this->x-b.x)+(this->y-b.y)*(this->y-b.y));
    int distance = b.distofgood.at(std::make_pair(x, y));
    if (distance <= dist)
    {
      this->dist = distance;
      this->berthid = b.id;
      return true;
    }
    return false;
  }
};