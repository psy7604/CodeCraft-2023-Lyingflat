//
// Created by daerh on 2023/3/11.
// header only
//

#ifndef CODECRAFTSDK_STRUCTURE_H
#define CODECRAFTSDK_STRUCTURE_H

#include <vector>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <ostream>
#include <iostream>
#include <unordered_set>


/**
 * @brief 物品类型
 */
struct ItemType {
    std::vector<int> formula;
    double purchasePrice;
    double originalSellingPrice;

    ItemType(const std::vector<int>& formula, double purchasePrice, double originalSellingPrice)
            : formula(formula),
              purchasePrice(purchasePrice),
              originalSellingPrice(originalSellingPrice) {}
};

/**
 * @brief 可用物品类型集合
 */
static const std::unordered_map<int, ItemType> itemTypeDict = {
        {1, ItemType(std::vector<int>{}, 3000, 4000)},
        {2, ItemType(std::vector<int>{}, 4400, 7600)},
        {3, ItemType(std::vector<int>{}, 5800, 9200)},
        {4, ItemType(std::vector<int>{1, 2}, 15400, 22500)},
        {5, ItemType(std::vector<int>{1, 3}, 17200, 25000)},
        {6, ItemType(std::vector<int>{2, 3}, 19200, 27500)},
        {7, ItemType(std::vector<int>{4, 5, 6}, 76000, 105000)}
};


struct WorktopType {
    std::vector<int> purchasingItemTypes;
    int producingItem;
    int workCycle;

    WorktopType(const std::vector<int>& purchasingItemTypes, int workCycle, int producingItemType)
            : purchasingItemTypes(purchasingItemTypes), producingItem(producingItemType), workCycle(workCycle) {}
};

static const std::unordered_map<int, WorktopType> worktopTypeDict = {
        {1, WorktopType(std::vector<int>{}, 50, 1)},
        {2, WorktopType(std::vector<int>{}, 50, 2)},
        {3, WorktopType(std::vector<int>{}, 50, 3)},
        {4, WorktopType(std::vector<int>{1, 2}, 500, 4)},
        {5, WorktopType(std::vector<int>{1, 3}, 500, 5)},
        {6, WorktopType(std::vector<int>{2, 3}, 500, 6)},
        {7, WorktopType(std::vector<int>{4, 5, 6}, 1000, 7)},
        {8, WorktopType(std::vector<int>{7}, 1, 0)},
        {9, WorktopType(std::vector<int>{1, 2, 3, 4, 5, 6, 7}, 1, 0)}
};

struct Vector2d;

struct Point {
    double x, y;

    Point(double x, double y) : x(x), y(y) {};

    Point operator+(const Point& p) const {
        return {x + p.x, y + p.y};
    }

    Point operator-(const Point& p) const {
        return {x - p.x, y - p.y};
    }

    Point operator+(const Vector2d& v);

    friend std::ostream& operator<<(std::ostream& os, const Point& point) {
        os << "(" << point.x << ", " << point.y << ")";
        return os;
    }
};

struct Vector2d {
    double x, y;

    Vector2d(double x, double y) : x(x), y(y) {}

    double Magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    double Orientation() const {
        return std::atan2(y, x);
    }

    Vector2d operator*(double d) {
        return {x * d, y * d};
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector2d& d) {
        os << "<" << d.x << ", " << d.y << ">";
        return os;
    }
};

inline Point Point::operator+(const Vector2d& v) {
    return {x + v.x, y + v.y};
}

struct Robot {
    static constexpr double radiusIdle = 0.45;
    static constexpr double radiusHolding = 0.53;
    static constexpr double assumedDensity = 20.0;
    static constexpr double assumedMaxForwardSpeed = 6.0;
    static constexpr double assumedMaxBackwardSpeed = 2.0;
    static constexpr double assumedMaxRotatingSpeed = M_PI;
    static constexpr double assumedMaxTractiveForce = 250.0;
    static constexpr double assumedMaxMoment = 50.0;


    // -1：表示当前没有处于任何工作台附近  [0,工作台总数-1] ：表示某工作台的下标，从 0 开始，按输入顺序定。
    // 当前机器人的所有购买、出售行为均针对该工作台进行。
    int worktopID = -1;

    // 携带物品类型 范围[0,7]。 0 表示未携带物品。 1-7 表示对应物品。
    int carryingItemType = 0;

    double timeValueCoefficient = 1.0;          // 时间价值系数
    double collisionValueCoefficient = 1.0;     // 碰撞价值系数

    // 角速度 单位：弧度/秒 正数：表示逆时针。负数：表示顺时针。
    double palstance = 0.0;

    // 线速度 2个浮点 x,y 由二维向量描述线速度，单位：米/秒
    Vector2d velocity = {0, 0};

    // 朝向 弧度，范围[-π,π]。方向示例：0：表示右方向。 π/2：表示上方向。 -π/2：表示下方向。
    double orientation = 0;

    Point position;                             // 位置

    explicit Robot(const Point& position) : position(position) {}

    double ItemPrice() const {
        if (carryingItemType != 0) {
            if (itemTypeDict.find(carryingItemType) != itemTypeDict.end()) {
                return timeValueCoefficient * collisionValueCoefficient *
                       itemTypeDict.find(carryingItemType)->second.originalSellingPrice;
            } else {
                return 0.0;
            }
        } else {
            return 0.0;
        }
    }

    void SellItem() {
        carryingItemType = 0;
    }

    void BuyItem(int itemType) {
        carryingItemType = itemType;
        timeValueCoefficient = 1.0;
        collisionValueCoefficient = 1.0;
    }

    /**
     * 半径
     * @return
     */
    double Radius() const {
        if (carryingItemType == 0) {
            return radiusIdle;
        } else {
            return radiusHolding;
        }
    }

    /**
     * 质量
     * @return
     */
    double Weight() const {
        return M_PI * Radius() * Radius() * assumedDensity;
    }

    /**
     * 转动惯量
     * @return
     */
    double J() const {
        return 0.5 * Weight() * Radius() * Radius();
    }

    /**
     * 角加速度
     * @return
     */
    double AngularAcceleration() const {
        return assumedMaxMoment / J();
    }

    /**
     * 加速度
     * @return
     */
    double Acceleration() const {
        return assumedMaxTractiveForce / Weight();
    }

    /**
     * 预测一定帧数后机器人的位置
     * @param frame 帧数
     * @param targetRotateSpeed 目标角速度(顺时针负，逆时针正）
     * @param targetVelocity 目标线速度
     * @param frameSkip 跳帧数
     * @return
     */
    std::vector<Point>
    PredictPosition(int frame, double targetRotateSpeed, double targetVelocity, int frameSkip) const {
        double timePerSkip = 1.0 / 50.0 * frameSkip;
        Point curPosition = position;
        double theta = orientation;
        double curPal = palstance;
        double curV = velocity.Magnitude();
        double angularAcc = AngularAcceleration();
        double acc = Acceleration();
        std::vector<Point> res;
        for (int curFrame = frameSkip; curFrame < frame; curFrame += frameSkip) {
            curPosition.x += curV * cos(theta) * timePerSkip;
            curPosition.y += curV * sin(theta) * timePerSkip;
            res.push_back(curPosition);
            if (curV < targetVelocity) {
                curV = std::min(targetVelocity, curV + timePerSkip * acc);
            } else if (curV > targetVelocity) {
                curV = std::max(targetVelocity, curV - timePerSkip * acc);
            }
            theta = theta + curPal * timePerSkip;
            if (curPal < targetRotateSpeed) {
                curPal = std::min(targetRotateSpeed, curPal + timePerSkip * angularAcc);
            } else if (curPal > targetRotateSpeed) {
                curPal = std::max(targetRotateSpeed, curPal - timePerSkip * angularAcc);
            }
        }
        return res;
    }

    void Refresh(const int& worktopID, const int& carryingItemType, const double& timeValueCoefficient,
                 const double& collusionCoefficient,
                 const double& palstance, const Vector2d& velocity, const double& orientation, const Point& position) {
        this->worktopID = worktopID;
        this->carryingItemType = carryingItemType;
        this->timeValueCoefficient = timeValueCoefficient;
        this->collisionValueCoefficient = collusionCoefficient;
        this->palstance = palstance;
        this->velocity = velocity;
        this->orientation = orientation;
        this->position = position;
    }

    friend std::ostream& operator<<(std::ostream& os, const Robot& robot) {
        os << "worktopID: " << robot.worktopID << " carryingItemType: " << robot.carryingItemType
           << " timeValueCoefficient: " << robot.timeValueCoefficient << " collisionValueCoefficient: "
           << robot.collisionValueCoefficient << " palstance: " << robot.palstance << " velocity: " << robot.velocity
           << " orientation: " << robot.orientation << " position: " << robot.position;
        return os;
    }
};

struct Worktop {
    Point position;                     // 位置
    int type;                           // 类型
    int remainingProductionTime = -1;   // 剩余生产时间
    int materialStatus = 0;             // 原材料格状态
    bool productionStatus = false;      // 产品格状态, true为当前有产品，false为无产品
    int purchasingItemBits = 0;
    const int producingItemType;        // 产出的产品，0表示不产出


    WorktopType Config() const {
        return worktopTypeDict.find(type)->second;
    }

    Worktop(const Point& position, int type) : position(position), type(type),
                                               producingItemType(Config().producingItem) {
        for (auto i: Config().purchasingItemTypes) {
            purchasingItemBits |= (0x1 << i);
        }
        // 如果不需要原材料，则即刻开始生产
        if (purchasingItemBits == 0) {
            remainingProductionTime = Config().workCycle;
        }
    }

    double ItemPrice() const {
        if (itemTypeDict.find(producingItemType) != itemTypeDict.end()) {
            return itemTypeDict.find(producingItemType)->second.purchasePrice;
        } else {
            return 0.0;
        }
    }

    /**
     * 工作台是否接受物品
     * @param itemType type
     */
    bool ItemAcceptable(int itemType) const {
        return itemType != 0 && ((purchasingItemBits & (1 << itemType)) != 0) &&
               ((materialStatus & (1 << itemType)) == 0);
    }

    /**
     * 工作台接受物品
     * @param itemType
     */
    void AcceptItem(int itemType) {
        materialStatus |= (1 << itemType);
        if (materialStatus == purchasingItemBits || purchasingItemBits == 0) {
            remainingProductionTime = Config().workCycle;
            materialStatus = 0;
        }
    }

    void SellItem() {
        productionStatus = false;
        if (materialStatus == purchasingItemBits) {
            remainingProductionTime = Config().workCycle;
            materialStatus = 0;
        }
    }

    bool Interactable(const Robot& robot) const {
        bool res = (robot.carryingItemType == 0 && productionStatus != 0) ||
                   ItemAcceptable(robot.carryingItemType);
        return res;
    }

    void Refresh(const int& type, const Point& position, const int& remainingProductionTime, const int& materialStatus,
                 const int& productionStatus) {
        this->type = type;
        this->position = position;
        this->remainingProductionTime = remainingProductionTime;
        this->materialStatus = materialStatus;
        this->productionStatus = productionStatus;
    }

    friend std::ostream& operator<<(std::ostream& os, const Worktop& worktop) {
        os << "position: " << worktop.position << " type: " << worktop.type << " remainingProductionTime: "
           << worktop.remainingProductionTime << " materialStatus: " << worktop.materialStatus << " productionStatus: "
           << worktop.productionStatus;
        return os;
    }
};

class Game;

extern std::vector<double> EstimateWorktops(const Game& gameStatus, const int robotIndex, int depth);

struct Task {
    double score;
    int worktopID;
};

class Assigner {
private:
    Game& game;
    std::unordered_set<int> availableSet;
    std::unordered_map<int, int> workDict;

public:
    explicit Assigner(Game& game) : game(game) {
    }

    /**
     * 读取地图后调用
     */
    void Init();


    int Size() {
        return (int) availableSet.size();
    }

    /**
     * 获取可用的任务
     * @param robotId 机器人ID
     * @return 若成功则返回score, worktopID，若失败则返回0.0, -1
     */
    Task AssignTask(int robotId) {
        static auto GetSortedIndex = [](const std::vector<double>& scores) -> std::vector<int> {
            int n = (int) scores.size();
            std::vector<int> indexs(n);
            for (int i = 0; i < n; i++) {
                indexs[i] = i;
            }
            for (int i = 1; i < n; i++) {
                int t = indexs[i];
                int j = i;
                while (j > 0 && scores[t] > scores[indexs[j - 1]]) {
                    j--;
                }
                for (int k = i; k > j; k--) {
                    indexs[k] = indexs[k - 1];
                }
                indexs[j] = t;
            }
            return indexs;
        };

        auto scores = EstimateWorktops(game, robotId, 0);

#ifdef _DEBUG
        std::cerr << "EstimateWorktops: " << std::endl;
        for (int i = 0; i < (int) scores.size(); i++) {
            std::cerr << "No." << i << " score is " << scores[i] << "  |  ";
        }
#endif

        auto sortedIndex = GetSortedIndex(scores);

#ifdef _DEBUG
        std::cerr << "sortedIndex: " << std::endl;
        for (auto i: sortedIndex) {
            std::cerr << i << ' ';
        }
        std::cerr << std::endl;
#endif

        for (auto i: sortedIndex) {
            if (this->availableSet.find(i) != this->availableSet.end()) {
                this->availableSet.erase(i);
                this->workDict[robotId] = i;
//                if (scores[i] < -500000.0) {
//                    return -1;
//                }
                return {scores[i], i};
            }
        }
        return {0.0, -1};
    }

    /**
     * 当前任务结束（不管是否成功）
     * @param robotId
     */
    void TaskOver(int robotId) {
        this->availableSet.insert(this->workDict[robotId]);
        this->workDict.erase(robotId);
    }

};

extern double Direction(const Point& p1, const Point& p2);

struct Game {
    static constexpr double mapSize = 50.0;
    int curFrame = 0;
    int money = 0;

    std::vector<Robot> robots;
    std::vector<Worktop> worktops;

    Assigner* assigner;

    Game() : assigner(new Assigner(*this)) {}

    Game(const Game& other) : curFrame(other.curFrame), money(other.money), robots(other.robots),
                              worktops(other.worktops), assigner(nullptr) {}


    /**
     * 初始化，读取地图后调用
     */
    void Init() {
        assigner->Init();
    }

    /**s
     * 刷新工作台在特定帧数后的状态
     * @param frames 帧数
     */
    void UpdateWorktops(int frames) {
        for (Worktop& w: worktops) {
            if (w.remainingProductionTime > frames) {
                w.remainingProductionTime -= frames;
            } else if (w.remainingProductionTime != -1) {
                w.remainingProductionTime = 0;
            }
            // 若当前产品格为空则刷新产品格
            if (w.remainingProductionTime == 0) {
                if (!w.productionStatus) {
                    w.productionStatus = true;
                    w.remainingProductionTime = -1;
                }
            }
            if (w.materialStatus == w.purchasingItemBits) {
                if (!w.productionStatus) {
                    w.remainingProductionTime = w.Config().workCycle;
                }
            }
        }
    }


    /**
     * 尝试进行交易
     * @param robotID
     * @param worktopID
     */
    void TryDoTrade(const int& robotID, const int& worktopID) {
        Worktop& worktop = worktops[worktopID];
        Robot& robot = robots[robotID];
        if (worktop.ItemAcceptable(robot.carryingItemType)) {
            worktop.AcceptItem(robot.carryingItemType);
            robot.SellItem();
            money += (int) robot.ItemPrice();
        }
        if (worktop.productionStatus && money > worktop.ItemPrice() && robot.carryingItemType == 0) {
            worktop.SellItem();
            robot.BuyItem(worktop.producingItemType);
            money -= (int) worktop.ItemPrice();
        }
    }

    void ApplySelection(const int& robotIndex, const int& worktopIndex) {
        Robot& curRobot = robots[robotIndex];
        Worktop& curWorktop = worktops[worktopIndex];

        curRobot.orientation = Direction(curRobot.position, curWorktop.position);
        curRobot.position = curWorktop.position;
        TryDoTrade(robotIndex, worktopIndex);
    }

    /**
     * @brief 为地图添加机器人
     * @param x x坐标
     * @param y y坐标
     *
     * 当地图上标记某个位置为机器人或者工作台时，则他们的坐标是该区域的中心坐标。
     * 地图第一行对应地图的最上方，最后一行对应地图的最下方,因此第一行第一列的中心
     * 坐标为：(0.25,49.75)。
     */
    void LoadRobot(double x, double y) {
        robots.emplace_back(Point(x, y));
    }

    /**
     * @brief 为地图添加工作台
     * @param x x坐标
     * @param y y坐标
     * @param type 工作台类型
     *
     * 当地图上标记某个位置为机器人或者工作台时，则他们的坐标是该区域的中心坐标。
     * 地图第一行对应地图的最上方，最后一行对应地图的最下方,因此第一行第一列的中心
     * 坐标为：(0.25,49.75)。
     */
    void LoadWorktop(double x, double y, int type) {
        worktops.emplace_back(Point(x, y), type);
    }

    void RefreshCurrentFrameID(int frameID) {
        this->curFrame = frameID;
    }

    /**
     * 刷新当前金钱
     * @param money 当前金钱数
     */
    void RefreshCurrentMoney(int curmoney) {
        this->money = curmoney;
    }

    /**
     * 刷新工作台状态
     * @param index 工作台序号
     * @param type 工作台类型
     * @param x 坐标x
     * @param y 坐标y
     * @param remainingProductionTime 剩余生产时间
     * @param materialStatus 原材料格状态
     * @param productionStatus 产品格状态
     */
    void RefreshWorktopStatus(int index, int type, double x, double y, int remainingProductionTime, int materialStatus,
                              int productionStatus) {
        worktops[index].Refresh(type, Point(x, y), remainingProductionTime, materialStatus, productionStatus);
    }

    /**
     * 刷新机器人状态
     * @param index 机器人索引
     * @param worktopID 所处工作台ID
     * @param carryingItemType 携带物品类型
     * @param timeCof 时间价值系数
     * @param collusionCof 碰撞价值系数
     * @param palstance 角速度
     * @param vx 线速度x
     * @param vy 线速度y
     * @param orientation 朝向
     * @param x 坐标x
     * @param y 坐标y
     */
    void
    RefreshRobotStatus(int index, int worktopID, int carryingItemType, double timeCof, double collusionCof,
                       double palstance,
                       double vx,
                       double vy, double orientation, double x, double y) {
        robots[index].Refresh(worktopID, carryingItemType, timeCof, collusionCof,
                              palstance, Vector2d(vx, vy), orientation, Point(x, y));
    }


    friend std::ostream& operator<<(std::ostream& os, const Game& game) {
        os << "curFrame: " << game.curFrame << " money: " << game.money << "\nRobots:\n";
        int i = 0;
        for (auto& p: game.robots) {
            os << "Robot No." << i << ":\n\t" << p << "\n";
            i++;
        }
        os << "\nWorktops:\n";
        i = 0;
        for (auto& p: game.worktops) {
            os << "Worktop No." << i << ":\n\t" << p << "\n";
            i++;
        }
        return os;
    }
};

inline void Assigner::Init() {
    this->availableSet = std::unordered_set<int>();
    for (int i = 0; i < (int) game.worktops.size(); i++) {
        this->availableSet.insert(i);
    }
}


#endif //CODECRAFTSDK_STRUCTURE_H
