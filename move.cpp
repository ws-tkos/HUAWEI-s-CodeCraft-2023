#include "move.h"
vector<position> nextpoints(4);
vector<vector<int>> occ_poss;
bool keep[4];
void cal_vec(position& next_pos, int rid)
{
    robot_state now_robot = robot_group[rid];
    double x_0 = now_robot.pos.x, y_0 = now_robot.pos.y;
    double x_t = next_pos.x, y_t = next_pos.y;
    double towards_0 = now_robot.direction; // 机器人朝向
    double towards_t = atan2(y_t - y_0, x_t - x_0); // 目标朝向
    double towards_margin, towards_margin_2, towards_margin_1 = towards_t - towards_0;
    // 计算顺时针、逆时针更快的转向目标的角度
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
    double result_rotate, result_forward;
    if (abs(towards_margin) < threshold_angle) {
        // 角度差异较小，直接冲过去
        // 1.计算角速度
        if (towards_margin < 0) {
            // 角度放大angle_ratio倍，尽快转弯
            result_rotate = max(-pi - 1, towards_margin * angle_ratio); // 设较大的数值，应对线上误差
        }
        else {
            result_rotate = min(pi + 1, towards_margin * angle_ratio);
        }
        // 角度过小时补足转弯速度
        if (abs(result_rotate) < 0.2) {
            result_rotate *= angle_ratio;
        }
        if (abs(result_rotate) < 0.05) {
            result_rotate *= angle_ratio;
        }
        // 2.计算线速度
        int now_direction_tag = (towards_t + pi) / (pi / 4); // 获取方向的数字化表示
        result_forward = max_Velocity_Line;
    }
    else {
        // 角度差异太大，停下来转角
        // 1.计算角速度
        if (towards_margin < 0) {
            // 角度放大angle_ratio倍，尽快转弯
            result_rotate = max(-pi - 1, towards_margin * angle_ratio); // 设较大的数值，应对线上误差
        }
        else {
            result_rotate = min(pi + 1, towards_margin * angle_ratio);
        }
        // 角度过小时补足转弯速度
        if (abs(result_rotate) < 0.2) {
            result_rotate *= angle_ratio;
        }
        if (abs(result_rotate) < 0.05) {
            result_rotate *= angle_ratio;
        }
        // 2.计算线速度
        result_forward = 0;
    }
    robot_group[rid].target_linear_velocity = result_forward;
    robot_group[rid].target_angular_velocity = result_rotate;
    // if(map_tag==2 && rid == 1)
    // {
    //     robot_group[rid].target_linear_velocity = 0;
    //     robot_group[rid].target_angular_velocity = 0;
    // }
}
void robotmoveControl()
{
    /*
     * 根据地图序号选择任务机制
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    for (int i = 0; i < 4; i++)
    {
        keep[i] = false;
        if (working_robot_group[i].now_state_tag == -1)
            continue;
        if (bot_miss_v[i].miss_cnt != 0)
        {
            bot_miss_v[i].miss_cnt--;
            if (bot_miss_v[i].type)
                cal_vec(bot_miss_v[i].pos, i);
            else
            {
                robot_group[i].target_linear_velocity = bot_miss_v[i].result_forward;
                robot_group[i].target_angular_velocity = bot_miss_v[i].result_rotate;
            }
            keep[i] = true;
            continue;
        }
        switch (move_method) {
        case 0: special_move_1(i);
            break;
        case 1: special_move_2(i);
            break;
        case 2: special_move_3(i);
            break;
        case 3: special_move_4(i);
            break;
        default:move(i);
            break;
        }
    }
    switch (avoid_crash_method) {
    case 0: special_avoid_crash_1();
        break;
    case 1: special_avoid_crash_2();
        break;
    case 2: special_avoid_crash_3();
        break;
    case 3: special_avoid_crash_4();
        break;
    default: avoid_crash();
        break;
    }
    switch (crash_method) {
    case 0: special_crash_1();
        break;
    case 1: special_crash_2();
        break;
    case 2: special_crash_3();
        break;
    case 3: special_crash_4();
        break;
    default: crash();
        break;
    }
    for (int i = 0; i < 4; i++)
    {
        printf("forward %d %f\n", i, robot_group[i].target_linear_velocity);
        printf("rotate %d %f\n", i, robot_group[i].target_angular_velocity);
    }
}
void avoid_crash()
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    for (int robot_ind = 0; robot_ind < 4; robot_ind++) {
        robot_state now_robot = robot_group[robot_ind];
        double direction_therold = 1.5;
        double now_crash_distance = 4;
        double now_crash_towards_margin = 0.6;
        //double direction_therold = 0.15;
        for (int i = 0; i < 4; i++) {
            if (i == robot_ind) continue;
            position p_robot_now = now_robot.pos;
            position p_robot_pre = robot_group[i].pos;
            int crash_tag; // 标识躲避策略
            double result_forward = -1, result_rotate = 0; // 结果
            if (robot_group[i].goods_id != -1 && robot_group[robot_ind].goods_id == -1) {
                // 前方是有货车辆时，本车是弱势方
                crash_tag = 1;
            }
            else if (robot_group[i].goods_id == -1 && robot_group[robot_ind].goods_id != -1) {
                // 前方是无货车辆时，且本车有货，本车是强势方
                crash_tag = 2;
            }
            else if (robot_group[i].goods_id == -1 && robot_group[robot_ind].goods_id == -1) {
                // 两车都空时，号码大的躲号码小的
                if (i < robot_ind) {
                    crash_tag = 1;
                }
                else {
                    crash_tag = 2;
                }
                //now_crash_distance /= 2;
                //now_crash_towards_margin /= 2;
            }
            else if (robot_group[i].goods_id != -1 && robot_group[robot_ind].goods_id != -1) {
                // 两车都满时，货物价值低的躲避货物价值高的
                if (goods_income[robot_group[i].goods_id] < goods_income[robot_group[robot_ind].goods_id]) {
                    crash_tag = 2;
                }
                else {
                    crash_tag = 1;
                }
            }
            // fprintf(stderr, "crash_tag: %d \n", crash_tag);
            double dis_e = cal_eucli_dis(p_robot_now, p_robot_pre);
            if (dis_e < now_crash_distance) {
                double x_0 = p_robot_now.x;
                double y_0 = p_robot_now.y;
                double x_t = p_robot_pre.x;
                double y_t = p_robot_pre.y;
                double towards_pre = atan2(y_t - y_0, x_t - x_0);
                double towards_0 = robot_group[robot_ind].direction;
                double towards_margin;
                double towards_margin_1 = towards_pre - towards_0;
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
                // 前方有机器人时
                if (abs(towards_margin) < now_crash_towards_margin) {
                    // 当前是慢车时，稍微降低警戒距离
                    if (robot_group[robot_ind].linear_velocity < 3 && dis_e > 3) {
                        continue;
                    }
                    double direction_margin;
                    double direction_a = robot_group[robot_ind].direction, direction_b = robot_group[i].direction;
                    if (direction_a > direction_b) {
                        double direction_temp = direction_a;
                        direction_a = direction_b;
                        direction_b = direction_temp;
                    }
                    double direction_margin_1 = direction_b - direction_a;
                    double direction_margin_2;
                    if (direction_margin_1 > 0) {
                        direction_margin_2 = direction_margin_1 - 2 * pi;
                    }
                    else {
                        direction_margin_2 = direction_margin_1 + 2 * pi;
                    }
                    if (abs(direction_margin_1) < abs(direction_margin_2)) {
                        direction_margin = direction_margin_1;
                    }
                    else {
                        direction_margin = direction_margin_2;
                    }
                    // 主动让行前方的直行车
                    if (abs(direction_margin) < direction_therold) {
                        crash_tag = 0;
                        result_forward = 0;
                        result_rotate = 0;
                        break;
                    }
                    else {
                        // 小角度的情况
                        if (direction_margin < 0) {
                            result_rotate = max(-pi + 0.05, direction_margin * angle_ratio);
                            result_forward = 3;
                            // result_rotate = -pi;
                        }
                        else {
                            result_rotate = min(pi - 0.05, direction_margin * angle_ratio);
                            result_forward = 3;
                            // result_rotate = pi;
                        }
                    }
                    result_rotate = -result_rotate;
                    //result_forward = 3;
                    if (result_forward != -1) {
                        if (crash_tag == 0) {
                            // 躲避大角度车，主动让行
                            //fprintf(stderr, "crash_1 : %f  %f\n", result_forward, result_rotate);
                            bot_miss_v[robot_ind].miss_cnt = 20;
                            bot_miss_v[robot_ind].result_forward = result_forward;
                            bot_miss_v[robot_ind].result_rotate = result_rotate;
                        }
                        else if (crash_tag == 1) {
                            // 躲避小角度车 主动退避一方的策略
                            //fprintf(stderr, "crash_2 : %f  %f\n", result_forward, result_rotate);
                            bot_miss_v[robot_ind].miss_cnt = 35;
                            bot_miss_v[robot_ind].result_forward = result_forward;
                            bot_miss_v[robot_ind].result_rotate = result_rotate;
                        }
                        else if (crash_tag == 2) {
                            // 躲避小角度车 强势一方的策略
                            //fprintf(stderr, "crash_2 : %f  %f\n", result_forward, result_rotate);
                            bot_miss_v[robot_ind].miss_cnt = 2;
                            bot_miss_v[robot_ind].result_forward = -2;
                            bot_miss_v[robot_ind].result_rotate = 0;
                        }
                        break; // 确定了策略，就跳出
                    }
                }
            }
        }
    }
}
void special_avoid_crash_1()
{

}
void special_avoid_crash_2()
{

}
void special_avoid_crash_3()
{

}
void special_avoid_crash_4()
{

}
int f[4];
int findf(int index)
{
    while (f[index]>=0)
        index = f[index];
    return index;
}
void merge(int index1, int index2)
{
    index1 = findf(index1);
    index2 = findf(index2);
    if (index1 != index2)
        f[index1] = index2;
}
void crash()
{
    /*
     * 普适性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    for (int i = 0; i < 4; i++)
        f[i] = -1;
    for (int i = 0; i < 4; i++)
        for (int j = i+1; j < 4; j++)
            if (cal_eucli_dis(robot_group[i].pos, robot_group[j].pos) < robot_group[i].r+ robot_group[j].r+0.05)
                merge(i, j);
    int cnt = 0;
    for (int i = 0; i < 4; i++)
        if(f[i]==-1)
        f[i]=-(++cnt);
    vector<vector<int>> crash_group(cnt,vector<int>());
    for (int i = 0; i < 4; i++)
    {
        if (keep[i])
            continue;
        int index = -f[findf(i)] - 1;
        while (crash_group.size() <= index)
            crash_group.push_back({});
        crash_group[index].push_back(i);
    }
    for(int i=0;i<crash_group.size();i++)
        if (crash_group[i].size()==2)
        {
            int index= crash_group[i][0];
            double maxprice = robot_group[crash_group[i][0]].crash_weight * robot_group[crash_group[i][0]].time_weight * (double)goods_sell_price[robot_group[crash_group[i][0]].goods_id] -(double)goods_buy_price[robot_group[crash_group[i][0]].goods_id];
            for (int j = 0; j < crash_group[i].size(); j++)
            {
                double price= robot_group[crash_group[i][j]].crash_weight * robot_group[crash_group[i][j]].time_weight * (double)goods_sell_price[robot_group[crash_group[i][j]].goods_id] - (double)goods_buy_price[robot_group[crash_group[i][j]].goods_id];
                if (price > maxprice)
                {
                    index = crash_group[i][j];
                    price = maxprice;
                }
            }
            //if (robot_group[index].goods_id == -1)
            //{
            //    gridpos vgpos = robot_group[index].vgpos;
            //    int x = vgpos.x;
            //    int y = vgpos.y;
            //    int xmv[4] = { 0,0,-1,-1 };
            //    int ymv[4] = { -1,0,-1,0 };
            //    for (int j = 0; j < 4; j++)
            //        if (x + xmv[j] >= 0 && x + xmv[j] < 100 && y + ymv[j] >= 0 && y + ymv[j] < 100)
            //            occ_map[x + xmv[j]][y + ymv[j]] = true;
            //}
            //else
            //{
            //    gridpos gpos = robot_group[index].vgpos;
            //    int x = gpos.x;
            //    int y = gpos.y;
            //    int xmv[9] = { -1,-1,-1,0,0,0,1,1,1};
            //    int ymv[9] = { -1,0,1,-1,0,1,-1,0,1};
            //    for (int j = 0; j < 9; j++)
            //        if (x + xmv[j] >= 0 && x + xmv[j] < 100 && y + ymv[j] >= 0 && y + ymv[j] < 100)
            //            occ_map[x + xmv[j]][y + ymv[j]] = true;
            //}
            for (int j = 0; j < crash_group[i].size(); j++)
            {
                if (crash_group[i][j] == index)
                    continue;
                    swap(robot_group[crash_group[i][j]], robot_group[index]);
                    move(index);
                    swap(robot_group[crash_group[i][j]], robot_group[index]);
                    cal_vec(nextpoints[index], crash_group[i][j]);
                    int angle1=atan2(robot_group[index].pos.y- robot_group[crash_group[i][j]].pos.y, robot_group[index].pos.x - robot_group[crash_group[i][j]].pos.x);
                    int angle2= atan2(nextpoints[index].y - robot_group[crash_group[i][j]].pos.y, nextpoints[index].x - robot_group[crash_group[i][j]].pos.x);
                    if (cal_angle_dis(angle1, angle2) <= pi / 2)
                    {
                        int gid = robot_group[crash_group[i][j]].goods_id;
                        robot_group[crash_group[i][j]].goods_id = robot_group[index].goods_id;
                        bool flag = false;
                        int tag;
                        if (working_robot_group[index].now_state_tag < 2 && cal_eucli_dis(robot_group[index].pos, desk_groups[working_robot_group[index].buy_did_dtp.dtp][working_robot_group[index].buy_did_dtp.did].pos) < robot_group[index].r +0.15)
                        {
                            flag = true;
                            tag = working_robot_group[index].now_state_tag;
                            working_robot_group[index].now_state_tag = 2;
                        }
                        swap(robot_group[crash_group[i][j]], robot_group[index]);
                        move(index);
                        swap(robot_group[crash_group[i][j]], robot_group[index]);
                        robot_group[crash_group[i][j]].goods_id = gid;
                        cal_vec(nextpoints[index], crash_group[i][j]);
                        if (flag)
                            working_robot_group[index].now_state_tag = tag;
                    }
            }
            // fprintf(stderr, "%d \n", index);
        }
        else if (crash_group[i].size()>2)
        {
            int index = crash_group[i][0];
            double maxprice = robot_group[crash_group[i][0]].crash_weight * robot_group[crash_group[i][0]].time_weight * (double)goods_sell_price[robot_group[crash_group[i][0]].goods_id] - (double)goods_buy_price[robot_group[crash_group[i][0]].goods_id];
            for (int j = 0; j < crash_group[i].size(); j++)
            {
                double price = robot_group[crash_group[i][j]].crash_weight * robot_group[crash_group[i][j]].time_weight * (double)goods_sell_price[robot_group[crash_group[i][j]].goods_id] - (double)goods_buy_price[robot_group[crash_group[i][j]].goods_id];
                if (price > maxprice)
                {
                    index = crash_group[i][j];
                    price = maxprice;
                }
            }
            for (int j = 0; j < crash_group[i].size(); j++)
            {
                if (crash_group[i][j] != index)
                {
                    bot_miss_v[crash_group[i][j]].miss_cnt = 15;
                    bot_miss_v[crash_group[i][j]].type = true;
                    bot_miss_v[crash_group[i][j]].result_forward = 0;
                    bot_miss_v[crash_group[i][j]].result_rotate = 0;
                }
            }
        }
}

void special_crash_1()
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_crash_2()
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
void special_crash_3()
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
void special_crash_4()
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
void move(int rid)
{
    /*
     * 移动方式一
     * param    rid     机器人编号
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    robot_state now_robot = robot_group[rid];
    working_robot_state now_working_robot = working_robot_group[rid];
    position target_pos;
    gridpos target_gpos;
    gridpos target_vgpos;
    bool flag = (now_robot.goods_id!=-1);
    // 获取目标位置
    if (now_working_robot.now_state_tag == 0 || now_working_robot.now_state_tag == 1) {
        // 买
        target_pos = desk_groups[now_working_robot.buy_did_dtp.dtp][now_working_robot.buy_did_dtp.did].pos;
        target_gpos = desk_groups[now_working_robot.buy_did_dtp.dtp][now_working_robot.buy_did_dtp.did].gpos;
        target_vgpos = desk_groups[now_working_robot.buy_did_dtp.dtp][now_working_robot.buy_did_dtp.did].vgpos;
        if (cal_eucli_dis(target_pos, now_robot.pos) < 0.4) {
            // 延迟购买状态下，目标位置转向卖方
            working_robot_group[rid].now_state_tag = 1;
            target_pos = desk_groups[now_working_robot.sell_did_dtp.dtp][now_working_robot.sell_did_dtp.did].pos;
            target_gpos = desk_groups[now_working_robot.sell_did_dtp.dtp][now_working_robot.sell_did_dtp.did].gpos;
            target_vgpos = desk_groups[now_working_robot.sell_did_dtp.dtp][now_working_robot.sell_did_dtp.did].vgpos;
        }
    }
    else {
        // 卖
        target_pos = desk_groups[now_working_robot.sell_did_dtp.dtp][now_working_robot.sell_did_dtp.did].pos;
        target_gpos = desk_groups[now_working_robot.sell_did_dtp.dtp][now_working_robot.sell_did_dtp.did].gpos;
        target_vgpos = desk_groups[now_working_robot.sell_did_dtp.dtp][now_working_robot.sell_did_dtp.did].vgpos;
    }
    position next_pos= now_robot.prepos;
    double nextdis = -1;
    // 获取bfs得出的下一个位置
    // fprintf(stderr, "zheli11\n");
    if (cal_eucli_dis(now_robot.pos, target_pos) < 0.9) {
        next_pos = target_pos;
    }
    else {
        vector<int> xmv = { -1,-1,-1,0,0,0,1,1,1 };
        vector<int> ymv = { -1,0,1,-1,0,1,-1,0,1 };
        if (flag)
        {
            for (int i = 0; i < xmv.size(); i++)
                if (now_robot.gpos.x + xmv[i] >= 0 && now_robot.gpos.x + xmv[i] < 100 + (!flag) && now_robot.gpos.y + ymv[i] >= 0 && now_robot.gpos.y + ymv[i] < 100 + (!flag))
                    if (robot_info[flag][now_robot.gpos.x + xmv[i]][now_robot.gpos.y + ymv[i]].
                        next_point_to_move.find(target_gpos.x * (100 + (!flag)) + target_gpos.y) != robot_info[flag][now_robot.gpos.x + xmv[i]][now_robot.gpos.y + ymv[i]].
                        next_point_to_move.end())
                        if (nextdis<0 || nextdis>robot_info[flag][now_robot.gpos.x + xmv[i]][now_robot.gpos.y + ymv[i]].short_dis_afterdij[target_gpos.x * (100 + (!flag)) + target_gpos.y])
                        {
                            nextdis = robot_info[flag][now_robot.gpos.x + xmv[i]][now_robot.gpos.y + ymv[i]].short_dis_afterdij[target_gpos.x * (100 + (!flag)) + target_gpos.y];
                            next_pos = gpos_to_pos(robot_info[flag][now_robot.gpos.x + xmv[i]][now_robot.gpos.y + ymv[i]].
                                    next_point_to_move[target_gpos.x * 100 + target_gpos.y][0]);
                        }
        }
        else
        {
            for (int i = 0; i < xmv.size(); i++)
                if (now_robot.vgpos.x + xmv[i] >= 0 && now_robot.vgpos.x + xmv[i] < 100 + (!flag) && now_robot.vgpos.y + ymv[i] >= 0 && now_robot.vgpos.y + ymv[i] < 100 + (!flag))
                    if (robot_info[flag][now_robot.vgpos.x + xmv[i]][now_robot.vgpos.y + ymv[i]].
                        next_point_to_move.find(target_vgpos.x * (100 + (!flag)) + target_vgpos.y) != robot_info[flag][now_robot.vgpos.x + xmv[i]][now_robot.vgpos.y + ymv[i]].
                        next_point_to_move.end())
                        if (nextdis<0 || nextdis>robot_info[flag][now_robot.vgpos.x + xmv[i]][now_robot.vgpos.y + ymv[i]].short_dis_afterdij[target_vgpos.x * (100 + (!flag)) + target_vgpos.y])
                        {
                            nextdis = robot_info[flag][now_robot.vgpos.x + xmv[i]][now_robot.vgpos.y + ymv[i]].short_dis_afterdij[target_vgpos.x * (100 + (!flag)) + target_vgpos.y];
                                next_pos = vgpos_to_pos(robot_info[flag][now_robot.vgpos.x + xmv[i]][now_robot.vgpos.y + ymv[i]].
                                    next_point_to_move[target_vgpos.x * 101 + target_vgpos.y][0]);
                        }
        }
    }
    now_robot.prepos = next_pos;
    nextpoints[rid] = next_pos;
    // fprintf(stderr, "%f %f %f\n", nextdis,next_pos.x, next_pos.y);
    cal_vec(next_pos, rid);
}

void special_move_1(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_move_2(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_move_3(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_move_4(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
