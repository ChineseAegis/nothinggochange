#pragma once
#include"MobileEquipment.hpp"
class Object:public MobileEquipment
{
    public:
  int money;
   Object(int x,int y,int money):MobileEquipment(x,y),money(money)
   {
    
   }

};