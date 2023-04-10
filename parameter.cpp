#include "parameter.h"
using namespace std;

//数学常量
 double pi=3.1415;

//其余信息
 long long money;//钱
 int frame_id;//帧数
 int all_frame =15000 ;
 double frame_t = 0.02;
 double cweight;//拥挤度权重
 int acftype;//激活函数类型

//地图相关信息
 int map_tag;//地图编号
 vector<string>  m(101, string(100, '0'));//地图
 bool gp_can_reach[100][100];//地图是否可达
 bool vgp_can_reach[101][101];//虚拟地图是否可达
 bool occ_map[101][101];//格子被占领
 vector<vector<int>> pos_belong_to_robot(50,vector<int>(50));//机器人可进入的地方
 int desknum;//站点数量
 desk_state desk_groups[10][50];//站点组
 int desk_cnt[10] = { 0 }; // 工作台计数
 int desk_need_goods_group[10]; // 可以在工作台售卖的商品类型用二进制表示

//工作台商品信息
 int goods_buy_price[8]= { 0, 3000,4400,5800,15400,17200,19200,76000 };;//商品买价格
 int goods_sell_price[8]= { 0, 6000,7600,9200,22500,25000,27500,105000 };;//商品卖价格
 int desk_product_tag[10][50][10]; // 买商品锁
 int desk_material_tag[10][50][10]; // 卖商品锁
 int desk_can_sell_goods_group[10][3] = {
    {-1, -1, -1},
    {4, 5, 9},
    {4, 6, 9},
    {5, 6, 9},
    {7, 9, -1},
    {7, 9, -1},
    {7, 9, -1},
    {8, 9, -1}
 };; // 可以售卖商品的工作台
 int goods_income[8];// 售卖物品的收入
 int now_goods_cnt[10];//现有商品计数

//机器人相关信息
 int robotnum=4;//机器人数量
 double bot_Density= 20; // 机器人密度
 robot_state robot_group[4];//机器人组
 bot_miss bot_miss_v[4]; // 机器人躲避状态记录

//机器人运动信息
 double max_Acceleration_Line_Loaded;
 double max_Acceleration_Line_UnLoaded;

 double velocity_Arrival = 2; // 进站目标速度
 double max_Velocity_Line = 7.0; // 最大线速度
 double threshold_angle = 0.3; // 停车调整的角度差阈值
 double traction_Force = 250; // 最大牵引力
 double radius_Loaded = 0.53; // 装载后机器人半径
 double radius_UnLoaded = 0.45; // 空机器人半径
 double angle_ratio = 10.0; // 小角度加速倍率
 double brake_therold = 0.3; // 刹车阈值
 int unloaded_miss_time = 40; // 空车躲避帧数
 int loaded_miss_time = 15; // 满车躲避帧数
 double crash_distance = 3; // 撞车探测距离
 double crash_towards_margin = 0.6; // 撞车探测角度
 int avoid_crash_method;
 int crash_method;
 int move_method;
//任务相关信息
 int cal_priority_mthod;
 vector<task> task_group;
//机器人和桌子关联信息(用于手调)
 int buy_desk_belong_to_robot[10][50];
 int sell_desk_belong_to_robot[10][50];
 vector<vector<int>>robot_move_ranges;
 int init_method;
 int assign_method;
//机器人和任务关联信息
 vector<working_robot_state> working_robot_group(4); // 正在运行的机器人
 robot_next_move_info robot_info[2][100][100];//机器人到达目标地点最短路径信息，第一个维度0表示未载物，1表示载物。


 //函数
 bool deskcmp(desk_state& d1, desk_state& d2)
 {
     if (d1.pos.y != d2.pos.y)
         return d1.pos.y < d2.pos.y;
     return d1.pos.x < d2.pos.x;
 }
