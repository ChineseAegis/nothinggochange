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
  int life=1000;
  Object(int x, int y, int money) : MobileEquipment(x, y), money(money)
  {
  }
  Object()
  {
    
  }
  
};