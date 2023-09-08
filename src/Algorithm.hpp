//
// Created by daerh on 2023/3/12.
//

// header only

#ifndef CODECRAFTSDK_ALGORITHM_HPP
#define CODECRAFTSDK_ALGORITHM_HPP

#include <cmath>
#include <memory>
#include <algorithm>
#include <unordered_set>
#include "Structure.hpp"
#include "GlobalSetting.h"

extern Game game;

inline double Distance(const Point& p1, const Point& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}


inline Vector2d operator-(const Point& p1, const Point& p2) {
    return {p2.x - p1.x, p2.y - p1.y};
}

/**
 * 创建从p1指向p2的向量
 */
inline Vector2d FromTo(const Point& p1, const Point& p2) {
    return {p2.x - p1.x, p2.y - p1.y};
}

//inline double Cross(const Vector2d& v1, const Vector2d& v2) {
//    return v1.x * v2.y - v2.x * v1.y;
//}

/**
 * 返回向量 p1->p2 和 p1->p3 的夹角，逆时针为正，顺时针为负
 */
//inline double BetweenAngle(const Point& p1, const Point& p2, const Point& p3) {
//    Vector2d v1 = FromTo(p1, p2);
//    Vector2d v2 = FromTo(p1, p3);
//    if (v1.Magnitude() == 0 || v2.Magnitude() == 0) {
//        return 0;
//    }
//    return std::asin(Cross(v1, v2) / (v1.Magnitude() * v2.Magnitude()));
//}

/**
 *  返回向量 p1->p2 的绝对角度
 */
inline double Direction(const Point& p1, const Point& p2) {
    return FromTo(p1, p2).Orientation();
}

/**
 * 返回两个角度的差值，a2在a1的顺时针方向为正，逆时针方向为负
 */
inline double AngleDiff(double a1, double a2) {
    double angleDiff = a1 - a2;
    if (std::abs(angleDiff) > M_PI) {
        if (angleDiff < 0.0) {
            angleDiff += 2.0 * M_PI;
        } else {
            angleDiff -= 2.0 * M_PI;
        }
    }
    return angleDiff;
}

/**
 * 返回机器人目前朝向与worktop方向之差，逆时针为正，顺时针为负
 * @param robot
 * @param worktop
 * @return
 */
inline double RobotWorktopAngleDiff(const Robot& robot, const Worktop& worktop) {
    double r2w = FromTo(robot.position, worktop.position).Orientation();
    const double& robotDir = robot.orientation;
    double angleDiff = robotDir - r2w;
    if (fabs(angleDiff) > M_PI) {
        if (angleDiff < 0.0) {
            angleDiff += 2.0 * M_PI;
        } else {
            angleDiff -= 2.0 * M_PI;
        }
    }
    return angleDiff;
}


inline double Estimate(const Game& gameStatus) {
    double res = 0.0;
    res -= gameStatus.curFrame * global::COST_PER_FRAME;
    res += gameStatus.money;
    for (const auto& r: gameStatus.robots) {
        res += r.ItemPrice();
    }
#ifdef _DEBUG
//    if (res == 0.0) {
//        std::cerr << gameStatus;
//    }
#endif
    return res;
}

/**
 * 估算特定游戏状态下，特定机器人抵达特定工作台所需要的帧数
 * @param gameStatus 游戏状态
 * @param robotIndex 机器人序号
 * @param worktopIndex 工作台序号
 */
inline int EstimateFrameCost(const Game& gameStatus, const int& robotIndex, const int& worktopIndex) {
    const Robot& curRobot = gameStatus.robots[robotIndex];
    const Worktop curWorktop = gameStatus.worktops[worktopIndex];

    // 这里估得少一点比较好
    int res = int(Distance(curRobot.position, curWorktop.position) /
                  (global::ASSUMED_ROBOT_VELOCITY * global::TIME_PER_FRAME) +
                  fabs(RobotWorktopAngleDiff(curRobot, curWorktop)) /
                  (global::ASSUMED_ROBOT_PALSTANCE * global::TIME_PER_FRAME));
    return res;
}

/**
 * 计算特定机器人完成特定任务后的游戏状态（用于评估）
 */



/**
 * 对于特定的机器人，递归地对所有工作台进行打分
 * @param gameStatus
 * @param robotIndex
 * @param worktopIndex
 * @param depth
 * @return
 */
inline std::vector<double> EstimateWorktops(const Game& gameStatus, const int robotIndex, int depth) {
    std::vector<double> res;

    for (int i = 0, n = (int) gameStatus.worktops.size(); i < n; i++) {
//        Game statusAfter = ApplySelection(gameStatus, robotIndex, i);
        Game statusAfter(gameStatus);
        auto frames = EstimateFrameCost(statusAfter, robotIndex, i);
        statusAfter.UpdateWorktops(frames);
        statusAfter.curFrame += frames;
        const Robot& robotAfter = statusAfter.robots[robotIndex];
        const Worktop& worktopAfter = statusAfter.worktops[i];
        if (worktopAfter.Interactable(robotAfter)) {
            statusAfter.ApplySelection(robotIndex, i);
            if (depth < global::SEARCH_DEPTH) {
                std::vector<double> v = EstimateWorktops(statusAfter, robotIndex, depth + 1);
                res.push_back(*std::max(v.begin(), v.end()));
            } else {
                double curScore = Estimate(statusAfter);
                res.push_back(curScore);
            }
        } else {
            res.push_back(-global::TOTAL_FRAMES * global::COST_PER_FRAME + (global::UNINTERACTABLE_PANELTY * 2) +
                          Distance(robotAfter.position, worktopAfter.position));
        }
    }

#ifdef _DEBUG
    if (depth == 0) {

    }
#endif

    return res;
}


#endif //CODECRAFTSDK_ALGORITHM_HPP
