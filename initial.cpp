#include <iostream>
#include <string>
#include <unordered_map>
#include <math.h>
#include <random>
#include <algorithm>
#include <queue>
#include <vector>
#include "initial.h"
#include "func.h"

using namespace std;
//初始化
void init()
{
    for (int i = 0; i < 4; i++)
    {
        working_robot_group[i].now_state_tag = -1;
    }
    init_robot_move_param();
}
void dij(bool flag,gridpos& dpos)//flag为0表示机器人不载物，flag为1表示机器人载物,dpos表示工作台格子坐标
{

    struct dis_dir_pos
    {
        double dis;
        int x;
        int y;
    };
    auto cmp = [](dis_dir_pos& a, dis_dir_pos& b) {return a.dis > b.dis; };
    vector<int> xmv = { -1,-1,-1,0,0,1,1,1 };
    vector<int> ymv = { -1,0,1,-1,1,-1,0,1 };
    //robot_info[2][8][100][100];//机器人到达目标地点最短路径信息，第一个维度0表示未载物，1表示载物。
    priority_queue<dis_dir_pos, vector<dis_dir_pos>, decltype(cmp)> q(cmp);
    q.push({ 0,dpos.x,dpos.y });
    if(flag)
    robot_info[flag][dpos.x][dpos.y].short_dis_afterdij[dpos.x * 100 + dpos.y] = 0;
    else
    robot_info[flag][dpos.x][dpos.y].short_dis_afterdij[dpos.x * 101 + dpos.y] = 0;
    int cnt = 0;
    while (!q.empty())
    {
        double dis = q.top().dis;
        int x = q.top().x;
        int y = q.top().y;
        gridpos gpos = { x,y };
        q.pop();
        if (!flag && dis > robot_info[flag][x][y].short_dis_afterdij[dpos.x * 101 + dpos.y])
            continue;
        if (flag&&dis > robot_info[flag][x][y].short_dis_afterdij[dpos.x * 100 + dpos.y])
            continue;
        for (int i = 0; i < 8; i++)
            if (x + xmv[i] >=0+ (!flag) && x + xmv[i] < 100 && y + ymv[i] >=0+ (!flag) && y + ymv[i] < 100)
            {
                gridpos nextgpos = { x + xmv[i],y + ymv[i] };
                if (flag)
                {
                    if (!gp_can_reach[x + xmv[i]][y + ymv[i]])
                        continue;
                }
                else
                {
                    if (!vgp_can_reach[x + xmv[i]][y + ymv[i]])
                        continue;
                }
                double nextdis = dis;
                if (abs(xmv[i]) + abs(ymv[i]) == 2)
                    nextdis += 0.5 * 1.414;
                else
                    nextdis += 0.5;
                if (robot_info[flag][x + xmv[i]][y + ymv[i]].next_point_to_move[dpos.x * (100+(!flag)) + dpos.y].empty() ||
                    nextdis < robot_info[flag][x + xmv[i]][y + ymv[i]].short_dis_afterdij[dpos.x * (100 + (!flag)) + dpos.y])
                {
                    cnt++;
                    q.push({ nextdis,x + xmv[i],y + ymv[i] });
                    robot_info[flag][x + xmv[i]][y + ymv[i]].next_point_to_move[dpos.x * (100 + (!flag)) + dpos.y].clear();
                    robot_info[flag][x + xmv[i]][y + ymv[i]].next_point_to_move[dpos.x * (100 + (!flag)) + dpos.y].push_back(gpos);
                    robot_info[flag][x + xmv[i]][y + ymv[i]].short_dis_afterdij[dpos.x * (100 + (!flag)) + dpos.y] = nextdis;
                }
                else if (abs(nextdis - robot_info[flag][x + xmv[i]][y + ymv[i]].short_dis_afterdij[dpos.x * (100 + (!flag)) + dpos.y]) <= 1e-9)
                    robot_info[flag][x + xmv[i]][y + ymv[i]].next_point_to_move[dpos.x * (100 + (!flag)) + dpos.y].push_back(gpos);
            }
    }
    // fprintf(stderr,"cnt %d\n", cnt);
}
void init_gp_can_reach()
{
    int xmv[9] = {-1,-1,-1,0,0,0,1,1,1};
    int ymv[9] = {-1,0,1,-1,0,1,-1,0,1};
    for(int i=0;i<100;i++)
        for (int j = 0; j < 100; j++)
        {
            gp_can_reach[i][j] = true;
            for (int k = 0; k < 9; k++)
                if (i + xmv[k] >= 0 && i + xmv[k] < 100 && j + ymv[k] >= 0 && j + ymv[k] < 100 && m[i + xmv[k]][j + ymv[k]] != '#')
                    continue;
                else
                {
                    gp_can_reach[i][j] = false;
                    break;
                }
        }
}
void init_vgp_can_reach()
{
    int xmv[4] = { 0,0,-1,-1 };
    int ymv[4] = {0,-1,0,-1};
    for (int i = 0; i < 101; i++)
        for (int j = 0; j < 101; j++)
        {
            if (i == 0 || j == 0 || i == 100 || j == 100)
            {
                vgp_can_reach[i][j] = false;
                continue;
            }
            vgp_can_reach[i][j] = true;
            for (int k = 0; k < 4; k++)
                if (i + xmv[k] >= 0 && i + xmv[k] < 101 && j + ymv[k] >= 0 && j + ymv[k] < 101 && m[i + xmv[k]][j + ymv[k]] != '#')
                    continue;
                else
                {
                    vgp_can_reach[i][j] = false;
                    break;
                }
        }
}
void init_robot_move_param() {
    double bot_Weight_Loaded;
    double bot_Weight_UnLoaded;
    bot_Weight_Loaded = pi * radius_Loaded * radius_Loaded * bot_Density;
    bot_Weight_UnLoaded = pi * radius_UnLoaded * radius_UnLoaded * bot_Density;
    max_Acceleration_Line_Loaded = traction_Force / bot_Weight_Loaded;
    max_Acceleration_Line_UnLoaded = traction_Force / bot_Weight_UnLoaded;
    init_gp_can_reach();
    init_vgp_can_reach();
    for(int i=0;i<100;i++)
        for (int j = 0; j < 100; j++)
        if(m[i][j]>='0'&&m[i][j]<='9')
        {
            gridpos gpos = { i,j };
            int vxmv[4] = {0,1,0,1};
            int vymv[4] = {0,0,1,1};
            int xmv[9] = { -1,-1,-1,0,0,0,1,1,1 };
            int ymv[9] = { -1,0,1,-1,0,1,-1,0,1};
            for (int k = 0; k < 4; k++)
                if (i + vxmv[k] >=0&&i + vxmv[k] < 100 &&j+vymv[k]>=0 && j + vymv[k] < 100)
                    if (vgp_can_reach[i + vxmv[k]][j + vymv[k]])
                    {
                        gpos.x += vxmv[k];
                        gpos.y += vymv[k];
                        dij(false, gpos);
                        gpos.x -= vxmv[k];
                        gpos.y -= vymv[k];
                    }
            bool flag = false;
            for (int k = 0; k < 9; k++)
                if (i + xmv[k] >= 0 && i + xmv[k] < 100 && j + ymv[k] >= 0 && j + ymv[k] < 100)
                    if (gp_can_reach[i + xmv[k]][j + ymv[k]])
                        flag = true;
            if (flag)
                dij(true, gpos);
        }
}
