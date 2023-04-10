#ifndef PARAMETER_H
#define PARAMETER_H
#include<iostream>
#include<unordered_map>
#include<vector>
#include<string>
#include<cmath>
#include<algorithm>
using namespace std;
/*结构体*/
//位置信息结构体
struct position
{
    double x;
    double y;
};
struct gridpos
{
    int x;
    int y;
};
//工作台信息结构体
struct desk_state//工作台信息
{
    int type;
    position pos;
    gridpos gpos;
    gridpos vgpos;
    double time;
    int material_state;
    int product_state;
};
struct desk_type_id
{
    int dtp;
    int did;
};
// 机器人信息结构体
struct robot_state//机器人信息
{
    int desk_id;
    int goods_id;
    double time_weight;
    double crash_weight;
    double angular_velocity;
    double linear_velocity;
    double direction;
    double weight;
    double r;
    double target_angular_velocity;
    double target_linear_velocity;
    position pos;
    position prepos;
    gridpos gpos;
    gridpos vgpos;
};
struct robot_next_move_info
{
    unordered_map<int,vector<gridpos>> next_point_to_move;
    unordered_map<int,double> short_dis_afterdij;
};
struct working_robot_state//机器人接取的任务
{
    // -1 闲置
    // 0 买的路上
    // 1 延迟购买
    // 2 卖的路上
    int now_state_tag;
    desk_type_id buy_did_dtp;
    desk_type_id sell_did_dtp;
    double finish_now_task_time;
    gridpos gpos;
    gridpos vgpos;
};
//运动信息
struct bot_miss// 躲避时的躲避时长和幅度
{
    bool type;
    double result_forward;
    double result_rotate;
    int miss_cnt;
    int miss_tag;
    position pos;
};
//任务结构体
struct task // 最小的买卖任务
{
    desk_type_id buy_did_dtp;
    desk_type_id sell_did_dtp;
    int goods_id;
    double task_priority;
    double price;
    double finish_time;
    gridpos gpos;
    gridpos vgpos;
};
/*实例*/

//数学常量
extern double pi;
//其余信息
extern long long money;//钱
extern int frame_id;//帧数
extern int all_frame;//所有帧
extern double frame_t;//一帧的时间
extern double cweight;//拥挤度权重
extern int acftype;//激活函数类型
//地图相关信息
extern int map_tag;//地图编号
extern vector<string> m;//地图
extern bool gp_can_reach[100][100];//地图是否可达
extern bool vgp_can_reach[101][101];//虚拟地图是否可达
extern bool occ_map[101][101];//格子被占领
extern vector<vector<int>> pos_belong_to_robot;//机器人可进入的地方
extern int desknum;//站点数量
extern desk_state desk_groups[10][50];//站点组
extern int desk_cnt[10]; // 工作台计数
extern int desk_need_goods_group[10]; // 可以在工作台售卖的商品类型用二进制表示
//工作台商品信息
extern int goods_buy_price[8];//商品买价格
extern int goods_sell_price[8];//商品卖价格
extern int desk_product_tag[10][50][10]; // 买商品锁
extern int desk_material_tag[10][50][10]; // 卖商品锁
extern int desk_can_sell_goods_group[10][3]; // 可以售卖商品的工作台
extern int goods_income[8];// 售卖物品的收入
extern int now_goods_cnt[10];//现有商品计数
//机器人相关信息
extern int robotnum;//机器人数量
extern double bot_Density; // 机器人密度
extern robot_state robot_group[4];//机器人组
extern vector<vector<int>>robot_move_ranges;
extern bot_miss bot_miss_v[4]; // 机器人躲避状态记录

//机器人运动信息
extern double max_Acceleration_Line_Loaded;
extern double max_Acceleration_Line_UnLoaded;
extern double velocity_Arrival; // 进站目标速度
extern double max_Velocity_Line; // 最大线速度
extern double threshold_angle; // 停车调整的角度差阈值
extern double traction_Force; // 最大牵引力
extern double radius_Loaded; // 装载后机器人半径
extern double radius_UnLoaded; // 空机器人半径
extern double angle_ratio; // 小角度加速倍率
extern double brake_therold; // 刹车阈值
extern int unloaded_miss_time; // 空车躲避帧数
extern int loaded_miss_time; // 满车躲避帧数
extern double crash_distance; // 撞车探测距离
extern double crash_towards_margin; // 撞车探测角度
extern int avoid_crash_method;
extern int crash_method;
extern int move_method;
//任务相关信息
extern int cal_priority_mthod;
extern vector<task> task_group;
//机器人和桌子关联信息(用于手调)
extern int buy_desk_belong_to_robot[10][50];
extern int sell_desk_belong_to_robot[10][50];
extern vector<vector<int>>robot_move_range;
extern int init_method;
extern int assign_method;

//机器人和任务关联信息
extern vector<working_robot_state> working_robot_group; // 正在运行的机器人
extern robot_next_move_info robot_info[2][100][100];//机器人到达目标地点最短路径信息，第一个维度0表示未载物，1表示载物。

/*函数*/
bool deskcmp(desk_state& d1, desk_state& d2);

#endif