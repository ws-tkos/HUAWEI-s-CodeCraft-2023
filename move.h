#ifndef MOVE_H
#define MOVE_H
#include <iostream>
#include <string>
#include <unordered_map>
#include <math.h>
#include <random>
#include <algorithm>
#include <queue>
#include <vector>
#include "parameter.h"
#include "func.h"
void robotmoveControl();
void avoid_crash();
void special_avoid_crash_1();
void special_avoid_crash_2();
void special_avoid_crash_3();
void special_avoid_crash_4();
void crash();
void special_crash_1();
void special_crash_2();
void special_crash_3();
void special_crash_4();

void move(int rid);
void special_move_1(int rid);
void special_move_2(int rid);
void special_move_3(int rid);
void special_move_4(int rid);
#endif
