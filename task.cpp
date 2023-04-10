#include <vector>
#include "task.h"

using namespace std;


void taskMechanismControl()
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
        if (map_tag == 2 &&(i ==0 || i == 1)) {
            working_robot_group[i].buy_did_dtp = { 8, 0 };
            working_robot_group[i].sell_did_dtp = { 9, 0 };
            working_robot_group[i].now_state_tag = 0;
            continue;
        }
        if (working_robot_group[i].now_state_tag != -1)
            continue;
        switch (init_method) {
        case 0: special_init_task_1(i);
            break;
        case 1: special_init_task_2(i);
            break;
        case 2: special_init_task_3(i);
            break;
        case 3: special_init_task_4(i);
            break;
        default: init_task(i);
            break;
        }
        switch (assign_method) {
        case 0: special_task_assign_1(i);
            break;
        case 1: special_task_assign_2(i);
            break;
        case 2: special_task_assign_3(i);
            break;
        case 3: special_task_assign_4(i);
            break;
        default:task_assign(i);
            break;
        }
    }
}
double cal_priority_Control(robot_state &robot,desk_state &buy_desk,desk_state &sell_desk)
{
    switch (cal_priority_mthod)
    {
        case 0: return special_cal_priority_1(robot,buy_desk,sell_desk);
        case 1: return special_cal_priority_2(robot, buy_desk, sell_desk);
        case 2: return special_cal_priority_3(robot, buy_desk, sell_desk);
        case 3: return special_cal_priority_4(robot, buy_desk, sell_desk);
        default:return cal_priority(robot, buy_desk, sell_desk);
    }
}
double special_cal_priority_1(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk)
{
    return 0;
}
double special_cal_priority_2(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk)
{
    return 0;
}
double special_cal_priority_3(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk)
{
    return 0;
}
double special_cal_priority_4(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk)
{
    return 0;
}
double cal_priority(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk)
{
    return 0;
}
void init_task(int rid)
{
    /*
     * 普适性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    task_group.clear();
    int xmv[4] = { 0,0,1,1 };
    int ymv[4] = { 0,1,0,1 };
    for (int now_desk_type = 7; now_desk_type > 0; now_desk_type--) {
        for (int i1 = 0; i1 < desk_cnt[now_desk_type]; i1++) {
            if ((buy_desk_belong_to_robot[now_desk_type][i1] & (1 << rid)) == 0)
                continue;
            desk_state now_desk_state = desk_groups[now_desk_type][i1];
            double now_robot_buy_dis = -1;
            gridpos vgpos;
            // fprintf(stderr, "init1\n");
            for(int index=0;index<4;index++)
                if (now_desk_state.vgpos.x + xmv[index] >= 0 && now_desk_state.vgpos.x + xmv[index] < 101 && now_desk_state.vgpos.y + ymv[index] >= 0 && now_desk_state.vgpos.y + ymv[index] < 101)
                {
                    now_desk_state.vgpos.x+= xmv[index];
                    now_desk_state.vgpos.y+= ymv[index];
                   double tmp= get_dij_dis(false, robot_group[rid].vgpos, now_desk_state.vgpos);
                   if (tmp >= 0)
                       if (now_robot_buy_dis<0 || now_robot_buy_dis>tmp)
                       {
                           vgpos = now_desk_state.vgpos;
                           now_robot_buy_dis = tmp;
                       }
                   now_desk_state.vgpos.x -= xmv[index];
                   now_desk_state.vgpos.y -= ymv[index];
                }
            if (now_robot_buy_dis < 0)
            {
                continue;
            }
            // fprintf(stderr, "init2\n");
            double t = now_robot_buy_dis / max_Velocity_Line * 50;
            if (now_desk_state.product_state == 0)
            {

                if (t + 2 * now_desk_state.type < now_desk_state.time || now_desk_state.time == -1)
                    continue;
            }
            if (now_desk_state.pos.x < robot_move_ranges[rid][0] || now_desk_state.pos.x > robot_move_ranges[rid][1])
            continue;
            if (now_desk_state.pos.y < robot_move_ranges[rid][2] || now_desk_state.pos.y > robot_move_ranges[rid][3])
                continue;

            // 购买方工作台的产品被其他任务占据，就跳过这个任务
            if (desk_product_tag[now_desk_type][i1][now_desk_type] != 0) {
                continue;
            }
            // 寻找所有当前商品能去的目标
            for (int i = 0; i < 3; i++) {
                if (desk_can_sell_goods_group[now_desk_type][i] <= 0) {
                    break;
                }
                int target_desk_type = desk_can_sell_goods_group[now_desk_type][i];
                for (int j1 = 0; j1 < desk_cnt[target_desk_type]; j1++) {
                    if ((sell_desk_belong_to_robot[target_desk_type][j1] & (1 << rid)) == 0)
                        continue;
                    desk_state target_desk_state = desk_groups[target_desk_type][j1];
                    int target_desk_buy_state = ((target_desk_state.material_state >> now_desk_type) & 1);


                    if (target_desk_state.pos.x < robot_move_ranges[rid][0] || target_desk_state.pos.x > robot_move_ranges[rid][1])
                    continue;
                    if (target_desk_state.pos.y < robot_move_ranges[rid][2] || target_desk_state.pos.y > robot_move_ranges[rid][3])
                        continue;
                    // 目标工作台这个商品格已经有商品，跳过这个任务
                    if (target_desk_buy_state == 1) {
                        continue;
                    }
                    if(desk_material_tag[target_desk_type][j1][now_desk_type]!=0)
                        continue;
                    double x_b = desk_groups[now_desk_type][i1].pos.x;
                    double y_b = desk_groups[now_desk_type][i1].pos.y;
                    double x_s = desk_groups[target_desk_type][j1].pos.x;
                    double y_s = desk_groups[target_desk_type][j1].pos.y;
                    double x_r = robot_group[rid].pos.x;
                    double y_r = robot_group[rid].pos.y;
                    double robot_buy_angle = atan2(y_b - y_r, x_b - x_r);
                    double robot_sell_angle = atan2(y_s - y_b, x_s - x_b);
                    double now_robot_sell_dis = get_dij_dis(true, now_desk_state.gpos, target_desk_state.gpos);
                    if (now_robot_sell_dis < 0)
                        continue;
                    int need_frame = ((now_robot_buy_dis+now_robot_sell_dis) / max_Velocity_Line ) * 50;
                    if (frame_id + need_frame + 100 > all_frame) {
                        continue;
                    }
                    task new_task;
                    new_task.buy_did_dtp.dtp = now_desk_type;
                    new_task.sell_did_dtp.dtp = target_desk_type;
                    new_task.buy_did_dtp.did = i1;
                    new_task.sell_did_dtp.did = j1;
                    new_task.goods_id = now_desk_type;
                    new_task.price= goods_income[now_desk_type];
                    new_task.task_priority = 0;
                    new_task.vgpos = vgpos;
                    new_task.gpos = target_desk_state.gpos;

                    double all_dis = ((now_robot_buy_dis + now_robot_sell_dis) / max_Velocity_Line) * 50;
                    //fprintf(stderr, "all_dis : %f \n", all_dis);
                    double now_loss = get_time_loss(all_dis);
                    new_task.price = (goods_sell_price[now_desk_type] * now_loss - goods_buy_price[now_desk_type]) / all_dis;

                    //double now_p = get_priority(now_desk_type, i1, target_desk_type, j1);
                    //new_task.price /= now_p;

                    //new_task.price = cal_priority_Control(robot_group[rid],desk_groups[now_desk_type][i1],desk_groups[target_desk_type][j1]);
                    new_task.price = get_price(now_desk_type, i1, target_desk_type, j1, new_task.price);
                    // if(desk_groups[now_desk_type][i1].pos.y < 25){
                    //     new_task.price *= 100;
                    // }
                    // if(desk_groups[target_desk_type][j1].pos.y < 25){
                    //     new_task.price *= 100;
                    // }
                    new_task.finish_time = all_dis;
                    task_group.push_back(new_task);
                }
            }
        }
    }
}

double get_price(int now_desk_type, int now_desk_id, int target_desk_type, int target_desk_id, double raw_price) {
    double complete_rate = 2; // 补全增益系数, 1是不增益, 越大越鼓励补全
    double ban_456_9_rate = 1; // 对于4、5、6，惩罚拿去9卖掉的系数，1是不增益，越大惩罚力度越大，小于1是鼓励卖给9
    double ban_123_9_rate = 1; // 对于1、2、3，惩罚拿去9卖掉的系数，1是不增益，越大惩罚力度越大，小于1是鼓励卖给9
    double loss_goods_rate= 2; // 缺货补齐系数，维持4、5、6的生产平衡，1是取消这个机制
    double from_456_to_7_rate = 1;
    double tag_7_rate = 1;

    switch (map_tag)// 缺货补齐系数，维持4、5、6的生产平衡，1是取消这个机制
    {
    case 1: loss_goods_rate = 2; break;
    case 2: loss_goods_rate = 1; break;
    case 3: loss_goods_rate = 1; break;
    case 4: loss_goods_rate = 1; break;
    default:
        break;
    }

    switch (map_tag)// 补全增益系数, 1是不增益, 越大越鼓励补全
    {
    case 1: complete_rate = 3; break;
    case 2: complete_rate = 1; break;
    case 3: complete_rate = 1; break;
    case 4: complete_rate = 1; break;
    default:
        break;
    }

    switch (map_tag)// 对于4、5、6，惩罚拿去9卖掉的系数，1是不增益，越大惩罚力度越大，小于1是鼓励卖给9
    {
    case 1: ban_456_9_rate = 1; break;
    case 2: ban_456_9_rate = 1; break;
    case 3: ban_456_9_rate = 1; break;
    case 4: ban_456_9_rate = 1; break;
    default:
        break;
    }

    switch (map_tag)// 对于1、2、3，惩罚拿去9卖掉的系数，1是不增益，越大惩罚力度越大，小于1是鼓励卖给9
    {
    case 1: ban_123_9_rate = 1; break;
    case 2: ban_123_9_rate = 1; break;
    case 3: ban_123_9_rate = 1; break;
    case 4: ban_123_9_rate = 1; break;
    default:
        break;
    }

    switch (map_tag)
    {
    case 1: from_456_to_7_rate = 1; break;
    case 2: from_456_to_7_rate = 1; break;
    case 3: from_456_to_7_rate = 1; break;
    case 4: from_456_to_7_rate = 1; break;
    default:
        break;
    }
    
    switch (map_tag)
    {
    case 1: tag_7_rate = 1; break;
    case 2: tag_7_rate = 1; break;
    case 3: tag_7_rate = 1; break;
    case 4: tag_7_rate = 1; break;
    default:
        break;
    }
    // 补全增益机制
    if (target_desk_type >= 4 && target_desk_type <= 7) {
        if (desk_groups[target_desk_type][target_desk_id].material_state > 0) {
            // if (map_tag == 1)
            // {
            //     // raw_price *= getcnt(desk_groups[target_desk_type][target_desk_id].material_state) * complete_rate;
            //     // fprintf(stderr, "%f ||", raw_price);
            //     raw_price *= getcnt(desk_groups[target_desk_type][target_desk_id].material_state);
            //     // fprintf(stderr, "%f\n", raw_price);
            // }
            // else
            raw_price *=complete_rate;
        }
    }

    // 惩罚4、5、6被直接卖掉的机制
    if (now_desk_type >= 4 && now_desk_type <= 6) {
        if (target_desk_type == 9) {
            raw_price /= ban_456_9_rate;
        }
    }
    // 惩罚1、2、3被直接卖掉的机制
    if (now_desk_type >= 1 && now_desk_type <= 3) {
        if (target_desk_type == 9) {
            raw_price /= ban_123_9_rate;
        }
    }
    // 刺激生产缺失的4、5、6
    if (target_desk_type == 4) {
        if (now_goods_cnt[4] < now_goods_cnt[5] && now_goods_cnt[4] < now_goods_cnt[6]) {
            //fprintf(stderr, "encourage_4 : %d  %d  %d \n", now_goods_cnt[4], now_goods_cnt[5], now_goods_cnt[6]);
            raw_price *= loss_goods_rate;
        }
    }
    if (target_desk_type == 5) {
        if (now_goods_cnt[5] < now_goods_cnt[6] && now_goods_cnt[5] < now_goods_cnt[4]) {
            //fprintf(stderr, "encourage_5 : %d  %d  %d \n", now_goods_cnt[4], now_goods_cnt[5], now_goods_cnt[6]);
            raw_price *= loss_goods_rate;
        }
    }
    if (target_desk_type == 6) {
        if (now_goods_cnt[6] < now_goods_cnt[4] && now_goods_cnt[6] < now_goods_cnt[5]) {
            //fprintf(stderr, "encourage_6 : %d  %d  %d \n", now_goods_cnt[4], now_goods_cnt[5], now_goods_cnt[6]);
            raw_price *= loss_goods_rate;
        }
    }

    if (target_desk_type == 7 && (now_desk_type == 4 || now_desk_type == 5 || now_desk_type == 6)) {
        //fprintf(stderr, " %f ", raw_price);
        raw_price *= from_456_to_7_rate;
        //fprintf(stderr, " %f \n", raw_price);
    }
    return raw_price;
}

void special_init_task_1(int rid)
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_init_task_2(int rid)
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
void special_init_task_3(int rid)
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
void special_init_task_4(int rid)
{
    /*
     * 图1特殊性任务调度机制
     * param    None
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}
bool task_cmp(task& a, task& b) {
    return a.price > b.price;
}
void task_assign(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
    // fprintf(stderr, "rid tsz %d %d\n", rid,task_group.size());
    if (task_group.size() == 0) {
        if (frame_id > all_frame-200) {
            working_robot_group[rid].now_state_tag = -1;
            robot_group[rid].target_angular_velocity = 0;
            robot_group[rid].target_linear_velocity = 0;
        }
        return;
    }
    sort(task_group.begin(), task_group.end(), task_cmp);
    task now_task;
    now_task = task_group[0];
    working_robot_group[rid].now_state_tag = 0;
    working_robot_group[rid].buy_did_dtp = now_task.buy_did_dtp;
    working_robot_group[rid].sell_did_dtp = now_task.sell_did_dtp;
    working_robot_group[rid].vgpos = now_task.vgpos;
    working_robot_group[rid].gpos = now_task.gpos;
    int now_buy_type = working_robot_group[rid].buy_did_dtp.dtp;
    int now_buy_id = working_robot_group[rid].buy_did_dtp.did;
    int now_sell_type = working_robot_group[rid].sell_did_dtp.dtp;
    int now_sell_id = working_robot_group[rid].sell_did_dtp.did;
    if (now_buy_type > 3) {
        //不要给1 2 3 加锁
        desk_product_tag[now_buy_type][now_buy_id][now_buy_type] = 1;
    }
    //desk_product_tag[now_buy_type][now_buy_id][now_buy_type] = 1;
    desk_material_tag[now_sell_type][now_sell_id][now_buy_type] = 1;

}

void special_task_assign_1(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_task_assign_2(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_task_assign_3(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}

void special_task_assign_4(int rid)
{
    /*
     * 在这里进行任务分配
     * param    map_num     地图序号（0-3）
     * return   None
     * Last update by ws
     * Last update on 2020-3-27-10:36
     */
}