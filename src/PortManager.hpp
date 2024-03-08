#pragma once
#include"Map.hpp"
#include"Berth.hpp"
#include"Ship.hpp"
#include<vector>
//管理地图，船，泊位，机器人的类
class PortManager
{
   Map map;
   std::vector<Berth> berthVector;//管理泊位的数组,索引就是泊位id
   std::vector<Ship> shipVector;//船的数组,索引就是船的id
   
};