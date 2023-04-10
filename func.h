#ifndef FUNC_H
#define FUNC_H
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
//激活函数

//信息读取与处理函数
bool readUntilOK();//读取OK
void readmap();//读取地图数据
void readmessage();//桌子与机器人信息读取
void init_now_goods_cnt();

//距离计算函数
double cal_eucli_dis(position& p1, position& p2);
double cal_angle_dis(double angle1,double angle2);
double get_dij_dis(bool flag, gridpos& p1, gridpos& p2);
double cal_robot_to_pos_dis(robot_state& r, position& pos);
//坐标转换相关函数
//坐标转换
position gpos_to_pos(gridpos& gpos);
gridpos pos_to_gpos(position& pos);
position vgpos_to_pos(gridpos& gpos);
gridpos pos_to_vgpos(position& pos);
//机器人相关函数
void update_robot_state();//机器人状态更新

//运动相关函数
position merge_vector(position& pos1, position& pos2);//向量合成
double get_time_loss(double x);
#endif