#pragma once
#include <list>
#include <unordered_map>
#include "PortManager.hpp"
#include "MobileEquipment.hpp"
#include "PairHash.hpp"
#include <set>
// 泊位

class Berth
{
public:
    int id;
    int x, y;     // 泊位左上角坐标
    int time;     // 到达虚拟点的时间
    int velocity; // 装载速度,指每帧可以装载的物品数
    int shipId = -1;
    int goods = 0;
    bool flag = 0;      // 判断是否分配成功
    double percent = 0; // 占比
    int vaild = 1;
    std::set<std::pair<int, int>> berthid_clo; // 与他最近的berthid,pair 里第一个是距离，第二个是id

    Berth()
    {
    }
    Berth(int id, int x, int y, int time, int velocity)
        : id(id), x(x), y(y), time(time), velocity(velocity) {}

    void ReadData(); // 从标准输入读取数据
};

void Berth::ReadData()
{
}