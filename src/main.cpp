#include <iostream>
#include"Map.hpp"
#include"PortManager.hpp"
using namespace std;

int main()
{

    PortManager *p=new PortManager;
    p->run();
    return 0;
}