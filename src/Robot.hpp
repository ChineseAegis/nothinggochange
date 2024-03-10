#pragma once
#include"MobileEquipment.hpp"

class Robot:public MobileEquipment
{
    public:
  int goods=0;//是否携带货物。0未携带，1携带
  int status=1;//状态，0表示恢复状态，1表示正常状态
  Robot(int x,int y,int goods,int status):MobileEquipment(x,y),goods(goods),status(status)
  {

  }
  Robot()
  {

  }

};