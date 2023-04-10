#ifndef INITIAL_H
#define INITIAL_H
#include <iostream>
#include <string>
#include <unordered_map>
#include <math.h>
#include <random>
#include <algorithm>
#include <queue>
#include <vector>
#include "parameter.h"

using namespace std;

//初始化
void init();
//地图相关信息初始化
void init_m_sum();
//机器人信息初始化
void init_param_bot_move();
//最短路径初始化相关函数
void init_robot_move_param();
void dij(bool flag, gridpos& dpos);
#endif //INITIAL_H
