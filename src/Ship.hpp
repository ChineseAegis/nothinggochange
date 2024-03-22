#pragma once
#include"MobileEquipment.hpp"
//船类//
class Ship:public MobileEquipment {
public:
    int id;//唯一id
    int capacity=-1; // 船的容积
    int goods_num=0;
    int status=2;//0 表示移动(运输)中 1 表示正常运行状态(即装货状态或运输完成状态)  2 表示泊位外等待状态
    int berthId=-1; //表示目标泊位，如果目标泊位是虚拟点，则为-1
    int myBerthId;
    int time;
    int myTime=0;
    int goback_counter=0;
    int back_status=0;
    Ship(int x,int y,int id,int capacity,int status,int berthId) : id(id),capacity(capacity),MobileEquipment(x,y),status(status),berthId(berthId) {}
    Ship()
    {

    }
};
