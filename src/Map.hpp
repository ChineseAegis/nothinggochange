#pragma once
#include <iostream>
#include <string>
#include<vector>
// 处理地图，存储地图数据
class Map
{
    // 地图数据（可参考 maps/*.txt）是一个 200 行 200 列的字符矩阵。地图数据中每个字符
    // 含义如下：
    // ‘.’ ： 空地
    // ‘*’ ： 海洋
    // ‘#’ ： 障碍
    // ‘A’ ： 机器人起始位置，总共 10 个。
    // ‘B’ ： 大小为 4*4，表示泊位的位置,泊位标号在后泊位处初始化。
    // ⚫ 保证机器人初始位置不会重叠。
    // ⚫ 保证泊位不会重叠。

    // 数据结构，存储信息
public:
    static const int WIDTH = 200;
    static const int HEIGHT = 200;
    //二维数组，代表地图中每个坐标的元素
    std::vector<std::vector<char>> grid;


    // 读取文件，将元素放入grid
    void readfile(std::string filename)
    {

    }
};