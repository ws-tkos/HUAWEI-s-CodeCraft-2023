#ifndef CODECRAFTSDK_TASK_H
#define CODECRAFTSDK_TASK_H
#include <vector>
#include "parameter.h"
#include "func.h"
using namespace std;
double cal_priority_Control(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk);
double special_cal_priority_1(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk);
double special_cal_priority_2(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk);
double special_cal_priority_3(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk);
double special_cal_priority_4(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk);
double cal_priority(robot_state& robot, desk_state& buy_desk, desk_state& sell_desk);
void taskMechanismControl();

void init_task(int rid);
void special_init_task_1(int rid);
void special_init_task_2(int rid);
void special_init_task_3(int rid);
void special_init_task_4(int rid);

void task_assign(int rid);
void special_task_assign_1(int rid);
void special_task_assign_2(int rid);
void special_task_assign_3(int rid);
void special_task_assign_4(int rid);

double get_price(int now_desk_type, int now_desk_id, int target_desk_type, int target_desk_id, double raw_price);

#endif //CODECRAFTSDK_TASK_H
