//
// Created by daerh on 2023/3/12.
//

#ifndef CODECRAFTSDK_GLOBALSETTING_H
#define CODECRAFTSDK_GLOBALSETTING_H

#include <cmath>

namespace global {
    static constexpr int FRAME_PER_SECOND = 50;

    static constexpr int TOTAL_FRAMES = 50 * 3 * 60;

    static constexpr int SEARCH_DEPTH = 0;


    static constexpr double TIME_PER_FRAME = 1 / (double) FRAME_PER_SECOND;

    // 索引从1开始
    static constexpr double ITEM_VALUES[] =
            {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    static constexpr double COST_PER_FRAME = 88.8;

    // 角度差到帧数的换算比
//    static constexpr double ANGLE_COEF = 1.2 / M_PI;

    // 假定的机器人线速度（用于评估）
    static constexpr double ASSUMED_ROBOT_VELOCITY = 6.0;

    // 假定的机器人角速度（用于评估）
    static constexpr double ASSUMED_ROBOT_PALSTANCE = M_PI;

    // 距离差到帧数的换算比
//    static constexpr double DISTANCE_COEF = 1.0 / 5.0;

    // 预测多少帧
    static constexpr int PREDICT_FRAMES = 24;

    // 预测时跳帧数
    static constexpr int PREDICT_FRAME_SKIP = 3;

    // 线速度在此之下则不在开根号
    static constexpr double VELOCITY_THRESHOLD = 0.7;

    // 角速度在此之下则不在开根号
    static constexpr double PALSTANCE_THRESHOLD = 0.0;

    // 无法互动的工作台分数惩罚
    static constexpr double UNINTERACTABLE_PANELTY = -50000000.0 - TOTAL_FRAMES * COST_PER_FRAME;

    // 无可互动工作台时，时间系数容忍下限
    static double TIME_COEF_THRESHOLD = 0.90;
}
#endif //CODECRAFTSDK_GLOBALSETTING_H
