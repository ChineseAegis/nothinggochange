#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
// 处理地图，存储地图数据
struct Element
{
    char type;
    int isRobot = -1;//-1代表当前坐标不存在机器人，否则int值为机器人id
    int isShip = -1;//同上
    int isBerth = -1;//同时
    int isObject = -1;
    Element(char type):type(type)
    {

    }

};
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
    // 二维数组，代表地图中每个坐标的元素
    std::vector<std::vector<Element>> grid;

    Map()
    {
    }

    // 读取文件，将元素放入grid
    void readfile(const std::string filename);
    // 从标准输入读取文件
    void ReadFromStandardInput();
};

void Map::readfile(const std::string filename)
{
    std::ifstream file(filename); // 打开文件
    if (!file.is_open())
    {
        std::cerr << "无法打开文件  ";
        return;
    }

    std::string line;
    while (std::getline(file, line))
    { // 逐行读取
        std::vector<Element> row;
        for (char c : line)
        {                     // 遍历行中的每个字符
            row.push_back(Element(c)); // 添加到当前行的 vector 中
        }
        grid.push_back(row); // 将当前行添加到 grid 中
    }

    file.close(); // 关闭文件
}

void Map::ReadFromStandardInput()
{
    std::string line;
    int lineCount = 0;

    // 读取地图数据：200行*200列
    while (lineCount < 200 && std::getline(std::cin, line))
    {
        // if (line.size() < 200)
        // {
        //     std::cerr << "错误:输入行的长度小于200个字符" << std::endl;
        //     return; // 或者处理错误
        // }
        std::vector<Element> row;
        for (int i = 0; i < 200; ++i)
        {
            row.push_back(Element(line[i]));
        }
        grid.push_back(row);
        ++lineCount;
    }
}