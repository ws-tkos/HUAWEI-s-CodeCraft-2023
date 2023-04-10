#include <iostream>
#include <string>
#include <unordered_map>
#include <math.h>
#include <random>
#include <algorithm>
#include <queue>
#include <vector>

#include "func.h"

using namespace std;


bool readUntilOK() {
    /*
     * ！！！系统方法，无需更改！！！
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
    }
    return false;
}


void readmap()
{
    /*
     * 读取地图数据，记录每个桌子的pos、类型；并统计各类型桌子数量
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    for (int i = 0; i < 101; i++)
    {
        cin >> m[i];
        for (int j = 0; j < m[i].length(); j++) {
            if (m[i][j] >= '0' && m[i][j] <= '9') {
                int desk_type_t = m[i][j] - '0';
                desknum++;
                desk_cnt[m[i][j] - '0']++;
            }

        }

    }
    // fprintf(stderr, "desknum: %d\n", desknum);
    switch(desknum)
    {
    case 9: map_tag = 1;break;
    case 27: map_tag = 2;
        // robot_move_ranges = { {0,26,0,40}, {0,50,0,50}, {0,50,0,50}, {20,50,0,40} };
        robot_move_ranges = {  {0,50,0,50}, {0,50,0,50} ,{0,26,0,40},{20,50,0,40}};
        break;
    default: map_tag = 0;break;
    }
}
//坐标转换
position gpos_to_pos(gridpos& gpos)
{
    return {0.5*(double)gpos.y+0.25,50-0.5 * (double)gpos.x-0.25};
}
gridpos pos_to_gpos(position& pos)
{
    return { 99-(int)(2*pos.y),(int)(2*pos.x) };
}
position vgpos_to_pos(gridpos& gpos)
{
    return { 0.5 * (double)gpos.y,50 - 0.5 * (double)gpos.x };
}
gridpos pos_to_vgpos(position& pos)
{
    return { max(1,100 - (int)(2 * pos.y + 0.5)),min(99,(int)(2 * pos.x+0.5))};
}
//读取信息
void readmessage()
{
    /*
     * 读取每一帧的信息
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    scanf("%d", &money);
    scanf("%d", &desknum);
    int now_desk_cnt[10] = { 0 };
    for (int i = 0; i < desknum; i++)
    {
        int desk_type;
        scanf("%d", &desk_type);
        //desk_type -= 1;
        int now_type_cnt = now_desk_cnt[desk_type];
        now_desk_cnt[desk_type]++;
        desk_groups[desk_type][now_type_cnt].type = desk_type;
        scanf("%lf %lf %lf %d %d", &desk_groups[desk_type][now_type_cnt].pos.x, &desk_groups[desk_type][now_type_cnt].pos.y, &desk_groups[desk_type][now_type_cnt].time,
            &desk_groups[desk_type][now_type_cnt].material_state, &desk_groups[desk_type][now_type_cnt].product_state);
        desk_groups[desk_type][now_type_cnt].gpos= pos_to_gpos(desk_groups[desk_type][now_type_cnt].pos);
        desk_groups[desk_type][now_type_cnt].vgpos = pos_to_vgpos(desk_groups[desk_type][now_type_cnt].pos);

    }
    for (int i = 1; i <= 9; i++) {
        sort(desk_groups[i], desk_groups[i] + desk_cnt[i], deskcmp);
    }
    for (int i = 0; i < robotnum; i++)
    {
        double temp_x, temp_y;
        scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &robot_group[i].desk_id, &robot_group[i].goods_id,
            &robot_group[i].time_weight, &robot_group[i].crash_weight,
            &robot_group[i].angular_velocity, &temp_x, &temp_y,
            &robot_group[i].direction,
            &robot_group[i].pos.x, &robot_group[i].pos.y);
        robot_group[i].goods_id -= 1;
        robot_group[i].weight = pi * 3.1415 * 0.45 * 0.45 * 20;
        robot_group[i].r = 0.45;
        if (robot_group[i].goods_id != -1)
        {
            robot_group[i].r = 0.53;
            robot_group[i].weight = pi * 3.1415 * 0.53 * 0.53 * 20;
        }
        robot_group[i].linear_velocity = sqrt(temp_x * temp_x + temp_y * temp_y);
        int now_robot_good_type = robot_group[i].goods_id;
        robot_group[i].gpos = pos_to_gpos(robot_group[i].pos);
        robot_group[i].vgpos = pos_to_vgpos(robot_group[i].pos);
    }
}
void init_now_goods_cnt() {
    /*
     * 初始现存商品数量
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    for (int i = 0; i < 10; i++) {
        now_goods_cnt[i] = 0;
    }
    // 产品位置上的
    for (int desk_type = 4; desk_type <= 6; desk_type++) {
        for (int j = 0; j < desk_cnt[desk_type]; j++) {
            if (desk_groups[desk_type][j].product_state == 1) {
                now_goods_cnt[desk_type] += 2;
            }
        }
    }
    // 7材料格上的
    for (int i = 4; i <= 6; i++) {
        for (int j = 0; j < desk_cnt[7]; j++) {
            if (((desk_groups[7][j].material_state >> i) & 1) == 1) {
                now_goods_cnt[i] += 2;
            }
        }
    }
    // 4 5 6 材料格上已经有的
    for (int desk_type = 4; desk_type <= 6; desk_type++) {
        for (int j = 0; j < desk_cnt[desk_type]; j++) {
            for (int i = 1; i <= 3; i++) {
                if (((desk_groups[desk_type][j].material_state >> i) & 1) == 1) {
                    now_goods_cnt[desk_type] += 1;
                }
            }
        }
    }
    // 4 5 6 材料格上没有，但是已经有机器人在做这个材料的生产的
    for (int desk_type = 4; desk_type <= 6; desk_type++) {
        for (int j = 0; j < desk_cnt[desk_type]; j++) {
            for (int i = 1; i <= 3; i++) {
                if ((((desk_groups[desk_type][j].material_state >> i) & 1) == 0) && desk_material_tag[desk_type][j][i] == 1) {
                    now_goods_cnt[desk_type] += 1;
                }
            }
        }
    }
}



position get_next_pos(int robot_ind, double next_frame) {
    /*
     * 获取下一个位置
     * param    robot_ind   机器人的序号
     * param    next_frame  下一个工作台的序号
     * return   返回位置
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    position now_pos = robot_group[robot_ind].pos;
    double line_v = robot_group[robot_ind].linear_velocity / 50;
    double theta = robot_group[robot_ind].direction;
    double new_x = now_pos.x + cos(theta) * line_v * next_frame;
    double new_y = now_pos.y + sin(theta) * line_v * next_frame;
    return position{new_x, new_y};
}

//机器人状态更新
void update_robot_state()
{
    /*
     * 更新机器人状态
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    for (int i = 0; i < 4; i++) {
        int now_working_state = working_robot_group[i].now_state_tag;
        
        if (now_working_state == -1) {
            continue;
        }
        if (now_working_state == 0 || now_working_state == 1) {
            int now_buy_type = working_robot_group[i].buy_did_dtp.dtp;
            int now_buy_id = working_robot_group[i].buy_did_dtp.did;
            if (cal_eucli_dis(robot_group[i].pos, desk_groups[now_buy_type][now_buy_id].pos) < 0.4 && robot_group[i].desk_id != -1 && desk_groups[now_buy_type][now_buy_id].product_state == 1) {
                if (now_working_state == 1) {
                    position next_pos = get_next_pos(i, 3);
                    if (cal_eucli_dis(next_pos, desk_groups[now_buy_type][now_buy_id].pos) > 0.4) {
                        cout << "buy " << i << '\n';
                        working_robot_group[i].now_state_tag = 2;
                        desk_product_tag[now_buy_type][now_buy_id][now_buy_type] = 0;
                    }
                }
            }
        }
        else if (now_working_state == 2) {
            int now_sell_type = working_robot_group[i].sell_did_dtp.dtp;
            int now_sell_id = working_robot_group[i].sell_did_dtp.did;
            int now_buy_type = working_robot_group[i].buy_did_dtp.dtp;
            double robot_dis = cal_eucli_dis(robot_group[i].pos, desk_groups[now_sell_type][now_sell_id].pos);            
            if (cal_eucli_dis(robot_group[i].pos, desk_groups[now_sell_type][now_sell_id].pos) < 0.4 && robot_group[i].desk_id != -1) {
                cout << "sell " << i << '\n';
                working_robot_group[i].now_state_tag = -1;
                desk_material_tag[now_sell_type][now_sell_id][now_buy_type] = 0;
                desk_groups[now_sell_type][now_sell_id].material_state |= (1 << now_buy_type);
                double now_crash_loss = (1 - robot_group[i].crash_weight) * goods_income[now_buy_type];

            }
        }
    }
}
double ac_sigmod(double num)
{
    /*
     * 更新拥挤度
     * param    num     函数自变量
     * return   对应的Sigmoid函数值
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    return 1.0 / (1.0 + pow(2.71, cweight * num));
}
double ac_softplus(double num)
{
    /*
     * 更新拥挤度
     * param    num     函数自变量
     * return   对应的Sigmoid函数值
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    return log(1.0+ pow(2.71, cweight * num));
}
double ac_tanh(double num)
{
    return (pow(2.71, cweight * num)- pow(2.71, -cweight * num))/ (pow(2.71, cweight * num) + pow(2.71, -cweight * num));
}

double get_dij_dis(bool flag, gridpos& p1, gridpos& p2)
{
    int x1 = p1.x;
    int x2 = p2.x;
    int y1 = p1.y;
    int y2 = p2.y;
    double rt = -1;
    if (robot_info[flag][x1][y1].short_dis_afterdij.find(x2 * (100 + (!flag)) + y2) != robot_info[flag][x1][y1].short_dis_afterdij.end())
            rt = (double)robot_info[flag][x1][y1].short_dis_afterdij[x2 * (100 + (!flag)) + y2];
    // add 
    //if (rt < 0)
    //{
    //    rt = INT_MAX;
    //    int xmv[5] = { 0, -1,0,1,0};
    //    int ymv[5] = { 0, 0,-1,0,1};
    //    for (int i = 0; i < 5; i++) {
    //        for (int j = 0; i < 5; i++) 
    //        {
    //            x1 = p1.x + xmv[i];
    //            y1 = p1.y + ymv[i];
    //            x2 = p2.x + xmv[i];
    //            y2 = p2.y + ymv[i];
    //            if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 || x1>100 + (!flag) || y1>100 + (!flag) || x2>100 + (!flag) || y2>100 + (!flag))
    //                continue;
    //            if (robot_info[flag][x1][y1].short_dis_afterdij.find(x2 * (100 + (!flag)) + y2) != robot_info[flag][x1][y1].short_dis_afterdij.end())
    //                rt = min((double)robot_info[flag][x1][y1].short_dis_afterdij[x2 * (100 + (!flag)) + y2], rt);
    //        }
    //    }
    //}
    return rt;
}
double cal_robot_to_pos_dis(robot_state& r, position& pos)
{
    double dis = cal_angle_dis(r.direction, atan2(pos.y - r.pos.y, pos.x - r.pos.x))/pi*max_Velocity_Line+cal_eucli_dis(r.pos,pos);
    return dis;
}
double cal_angle_dis(double angle_a, double angle_b) {
    if (angle_a > angle_b) {
        double angle_temp = angle_a;
        angle_a = angle_b;
        angle_b = angle_a;
    }
    double towards_margin;
    double towards_margin_1 = angle_a - angle_b;
    double towards_margin_2;
    if (towards_margin_1 > 0) {
        towards_margin_2 = towards_margin_1 - 2 * pi;
    }
    else {
        towards_margin_2 = towards_margin_1 + 2 * pi;
    }

    if (abs(towards_margin_1) < abs(towards_margin_2)) {
        towards_margin = towards_margin_1;
    }
    else {
        towards_margin = towards_margin_2;
    }
    double result = abs(towards_margin);
    return result;
}
double cal_eucli_dis(position &pos1,position &pos2)
{
    return sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x)+ (pos1.y - pos2.y) * (pos1.y - pos2.y));
}
double get_time_loss(double x) {
    double minRate = 0.8;
    double maxX = 9000;
    double result = minRate + (1 - minRate) * (1 - sqrt(1 - (1 - x / maxX) * (1 - x / maxX)));
    return result;
}

