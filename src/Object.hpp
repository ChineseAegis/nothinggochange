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
  
};