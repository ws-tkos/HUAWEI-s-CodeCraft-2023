#include <iostream>
#include<string>
#include<vector>
#include<unordered_map>
#include "parameter.h"
#include "func.h"
#include "initial.h"
#include "task.h"
#include "move.h"
using namespace std;

int main() {
    // 读取地图数据
    robot_move_ranges = { {0,50,0,50},{0,50,0,50}, {0,50,0,50}, {0,50,0,50} };
    readmap();
    for (int i = 0; i < 10; i++)
        for(int j=0;j<50;j++)
    {
            buy_desk_belong_to_robot[i][j] = 15;
            sell_desk_belong_to_robot[i][j] = 15;
    }
    init();
    init_method = 4;
    assign_method = 4;
    move_method = 4;
    crash_method = 4;
    avoid_crash_method = 4;
    printf("OK\n");
    fflush(stdout);
    fflush(stdin);
    while (scanf("%d", &frame_id) != EOF) {
        // 读取每帧信息
        readmessage();
        readUntilOK();
        fflush(stdout);
        printf("%d\n", frame_id);
        // 任务分配
        update_robot_state();
        init_now_goods_cnt();
        // 移动指令
        taskMechanismControl();
        robotmoveControl();
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}