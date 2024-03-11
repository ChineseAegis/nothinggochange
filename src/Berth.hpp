#pragma once
#include<list>
#include<unordered_map>
#include"PortManager.hpp"
#include"MobileEquipment.hpp"
#include"PairHash.hpp"
//泊位
class Berth {
public:
    int id;
    int x, y; // 泊位左上角坐标
    int time; // 到达虚拟点的时间
    int velocity; // 装载速度,指每帧可以装载的物品数
    std::list<std::pair<int,int>>path_of_move;
    std::unordered_map<std::pair<int,int>,std::vector<MobileEquipment>,pair_hash> pathofgood;
    std::unordered_map<std::pair<int,int>,int,pair_hash> distofgood;
    Berth(){

    }
    Berth(int id, int x, int y, int time, int velocity)
        : id(id), x(x), y(y), time(time), velocity(velocity) {}

    void ReadData();//从标准输入读取数据
};

void Berth::ReadData()
{
   
}