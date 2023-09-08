//
// Created by daerh on 2023/3/17.
//

#ifndef CODECRAFTSDK_ROBOTCONTROL_HPP
#define CODECRAFTSDK_ROBOTCONTROL_HPP

#include <unordered_map>
#include <memory>
#include <ostream>
#include <sstream>

#include "Structure.hpp"
#include "Algorithm.hpp"

#ifdef _DEBUG

#include <windows.h>

#endif

struct Event {
};

struct GetTarget : public Event {
    explicit GetTarget(int target) : target(target) {}

    int target;
};

struct Blocked : public Event {
    Blocked() {}
};

struct Unblocked : public Event {
    Unblocked() {}
};

struct Done : public Event {
    Done() {}
};

struct Abandon : public Event {
    Abandon() {}
};

struct RobotController;

struct RobotState {

    virtual void Update(RobotController* controller) = 0;

    void React(const Event& e, RobotController*) {};

    virtual void React(const GetTarget&, RobotController*) {};

    virtual void React(const Blocked&, RobotController*) {};

    virtual void React(const Unblocked&, RobotController*) {};

    virtual void React(const Done&, RobotController*) {};

    virtual void React(const Abandon&, RobotController*) {};

    virtual std::string ToString() {
        return "RobotState";
    }
};


struct Assign : RobotState {
    void Update(RobotController* controller) override;

    void React(const GetTarget& t, RobotController* controller) override;

    void React(const Abandon& a, RobotController* controller) override;

    std::string ToString() override {
        return "Assign";
    }

    static Assign& Instance() {
        static Assign singleton;
        return singleton;
    }
};

struct Pathfind : RobotState {
    void Update(RobotController* controller) override;

    void React(const Blocked&, RobotController*) override;

    void React(const Abandon&, RobotController*) override;

    void React(const Done&, RobotController*) override;

    std::string ToString() override {
        return "Pathfind";
    }

    static Pathfind& Instance() {
        static Pathfind singleton;
        return singleton;
    }
};

struct Avoid : RobotState {
    void Update(RobotController* controller) override;

    void React(const Unblocked&, RobotController*) override;

    std::string ToString() override {
        return "Pathfind";
    }

    static Avoid& Instance() {
        static Avoid singleton;
        return singleton;
    }
};

struct Instruction {
    enum class Type {
        forward,
        rotate,
        buy,
        sell,
        destroy,
    } type;
    int robotID;
    double value;

    std::string ToString() const {
        std::stringstream ss;
        switch (type) {
            case Type::forward:
                ss << "forward";
                break;
            case Type::rotate:
                ss << "rotate";
                break;
            case Type::buy:
                ss << "buy";
                break;
            case Type::sell:
                ss << "sell";
                break;
            case Type::destroy:
                ss << "destroy";
                break;
            default:

                break;
        }
        ss << ' ';
        ss << robotID;
        if (type == Type::forward || type == Type::rotate) {
            ss << ' ' << value;
        }
        return ss.str();
    }
};

struct RobotController {
private:
    RobotState* curState;
    Game& game;
    int robotIndex;
    int curTargetWorktopID;
    std::vector<Instruction> instructionCache;

public:
    explicit RobotController(Game& game, int robotIndex)
    // 初始状态为Assign
            : curState(&Assign::Instance()), game(game), robotIndex(robotIndex), curTargetWorktopID(-1),
              instructionCache() {
    }

    Game& GameStatus() const {
        return game;
    }

    int RobotIndex() const {
        return robotIndex;
    }

    int GetTarget() const {
        return curTargetWorktopID;
    }

    Robot& GetRobot() {
        return game.robots[robotIndex];
    }

    RobotState& GetCurState() {
        return *curState;
    }

    const std::vector<Instruction>& GetInstructionCache() {
        return instructionCache;
    }

    void ClearInstructionCache() {
        instructionCache.clear();
    }

    /**
     * 由总控制器调用
     */
    void Update() {
        curState->Update(this);
    }

private:
    /**
     * 内部函数，使机器人导航到某位置
     * @param target 目标点
     */
    void GuideTo(const Point& target) {
        const Robot& curRobot = GetRobot();
        Vector2d DistanceVector = FromTo(curRobot.position, target);//距离向量
        double DisVecForward = atan2(DistanceVector.y, DistanceVector.x);     //距离向量的方向
        double theta = AngleDiff(curRobot.orientation, DisVecForward);  //应该转的角度
        double beta = curRobot.AngularAcceleration();  //角加速度
        double tpal = theta * beta;
        double omega;
        if (fabs(tpal) > global::PALSTANCE_THRESHOLD) {
            omega = tpal >= 0.0 ? -sqrt(tpal) : sqrt(-tpal);  //角速度
        } else {
            omega = tpal >= 0.0 ? -tpal : tpal;  //角速度
        }


        double Distance = DistanceVector.Magnitude();
        double tval = 2.0 * curRobot.Acceleration() * Distance;
        double theoMaxVector = sqrt(tval); //理论上的最大速度
        double maxVector = theoMaxVector < 6.0 ? theoMaxVector : 6.0;

        if (fabs(theta) > (M_PI / 3)) {
            maxVector = 0.0;
            omega = tpal >= 0.0 ? -M_PI : M_PI;
        }

        instructionCache.push_back(Instruction{
                .type = Instruction::Type::rotate,
                .robotID = robotIndex,
                .value = omega,
        });

        instructionCache.push_back(Instruction{
                .type = Instruction::Type::forward,
                .robotID = robotIndex,
                .value = maxVector,
        });
    }

    void AbandonItem() {
        if (GetRobot().carryingItemType != 0) {
            instructionCache.push_back(Instruction{
                    .type = Instruction::Type::destroy,
                    .robotID = robotIndex,
                    .value = 0.0,
            });
        }
    }

public:
    // 外部控制接口，由状态调用

    /**
     * 控制接口，转换当前状态
     * @param instance 状态的单例，由Instance()获得
     */
    void TransitState(RobotState& instance) {
        curState = &instance;
    }

    /**
     * 从全局分配器获取下一个任务
     * @return 工作台编号，若为-1则表示分配失败
     */
    Task GetAssignedTask() const {
        return GameStatus().assigner->AssignTask(RobotIndex());
    }

    /**
     * 停止任务（无论成功或失败）
     * @return
     */
    void TerminateTask() {
        GameStatus().assigner->TaskOver(RobotIndex());
        curTargetWorktopID = -1;
    }

    /**
     * 控制接口，设置目标工作台，若参数为-1表示清空当前目标
     * @param worktopID
     */
    void SetTargetWorktop(int worktopID) {
        curTargetWorktopID = worktopID;

#ifdef _DEBUG
        std::cerr << "selected target No." << worktopID << std::endl;
        std::cerr << "target worktop:\n" << GameStatus().worktops[worktopID] << std::endl;
#endif
    }

    /**
     * 控制接口，使机器人继续移动
     */
    void ContinueMoving() {
        GuideTo(game.worktops[curTargetWorktopID].position);
    }

    /**
     * 控制接口，机器人尝试进行交易（买卖同步）
     */
    bool TryDoTrade() {
        if (GetRobot().carryingItemType == 0) {
            instructionCache.push_back(Instruction{
                    .type = Instruction::Type::buy,
                    .robotID = robotIndex,
                    .value = 0.0
            });
            return true;
        } else if (GameStatus().worktops[curTargetWorktopID].ItemAcceptable(GetRobot().carryingItemType)) {
            instructionCache.push_back(Instruction{
                    .type = Instruction::Type::sell,
                    .robotID = robotIndex,
                    .value = 0.0
            });
            instructionCache.push_back(Instruction{
                    .type = Instruction::Type::buy,
                    .robotID = robotIndex,
                    .value = 0.0
            });
            return true;
        } else {
            return false;
        }
    }

    bool ReachTarget() {
        return GetRobot().worktopID == curTargetWorktopID;
    }

    void Abandon() {
        AbandonItem();
    }

    bool NotStucked() {
        const auto& status = GameStatus();
        for (auto& w: status.worktops) {
            if (w.Interactable(GetRobot())) {
                return true;
            }
        }
        return false;
    }

    bool Blocked() {
        // TODO: 增加障碍判断
        return false;
    }
};

void Assign::Update(RobotController* controller) {
#ifdef _DEBUG
    std::cerr << "-----------------------------------------------------------------------------------------"
              << std::endl;
    std::cerr << "ROBOT: No." << controller->RobotIndex() << std::endl << controller->GetRobot() << std::endl;
    std::cerr << "Assign::Update: " << std::endl;
#endif
    auto t = controller->GetAssignedTask();
    double score = t.score;
    int target = t.worktopID;
    if (target != -1) {
#ifdef _DEBUG
        std::cerr << "timeValueCoefficient: " << controller->GetRobot().timeValueCoefficient << std::endl;
#endif
        if (controller->GetRobot().timeValueCoefficient > 0.79 &&
            controller->GetRobot().timeValueCoefficient < global::TIME_COEF_THRESHOLD) {
#ifdef _DEBUG
            std::cerr << "Abandon at score: " << score << " timeValueCoefficient: "
                      << controller->GetRobot().timeValueCoefficient << std::endl;
#endif
            controller->GetCurState().React(Abandon(), controller);
        } else {
            controller->GetCurState().React(GetTarget(target), controller);
        }
    }

#ifdef _DEBUG
    static long long count = 0;
    if (count < 4) {
        Sleep(500);
    } else {
        Sleep(500);
    }
    count++;
#endif
}

void Assign::React(const GetTarget& t, RobotController* controller) {
    controller->SetTargetWorktop(t.target);
    controller->TransitState(Pathfind::Instance());
}

void Assign::React(const Abandon& a, RobotController* controller) {
#ifdef _DEBUG
    std::cerr << "Abandon! " << "at assigner size: " << controller->GameStatus().assigner->Size();

#endif
    controller->TerminateTask();
    controller->Abandon();
    controller->TransitState(Assign::Instance());
}

void Pathfind::Update(RobotController* controller) {
    if (controller->ReachTarget()) {
        controller->GetCurState().React(Done{}, controller);
    } else if (controller->Blocked()) {
        // TODO: 增加避障处理

    } else {
        controller->ContinueMoving();
        if (!controller->NotStucked()) {
            if (controller->GetRobot().timeValueCoefficient < global::TIME_COEF_THRESHOLD) {
                controller->GetCurState().React(Abandon(), controller);
            }
        }
    }
}

void Pathfind::React(const Abandon& a, RobotController* controller) {
    controller->TerminateTask();
    controller->Abandon();
    controller->TransitState(Assign::Instance());
}

void Pathfind::React(const Blocked&, RobotController* controller) {
    controller->TerminateTask();
    // TODO: 增加避障处理
}

void Pathfind::React(const Done&, RobotController* controller) {
    if (controller->TryDoTrade()) {
#ifdef _DEBUG
        std::cerr << "Trading instructions: ";
        for (const auto& i: controller->GetInstructionCache()) {
            std::cerr << i.ToString() << " ";
        }
        std::cerr << std::endl;
#endif
    }
    controller->TerminateTask();
    controller->TransitState(Assign::Instance());
}

void Avoid::Update(RobotController* controller) {
    // TODO: 增加避障处理
}

void Avoid::React(const Unblocked&, RobotController*) {
    // TODO: 增加避障处理
}

/**
 * 总控制器（必须在读取地图之后进行初始化，即调用Init）
 */
struct GeneralController {
    Game& game;
    std::vector<RobotController> controllers;

    explicit GeneralController(Game& game) : game(game) {}

    /**
     * 初始化，在读取地图之后，开始游戏之前调用
     */
    void Init() {
        for (int i = 0; i < (int) game.robots.size(); i++) {
            controllers.emplace_back(game, i);
        }
    }

    /**
     * 刷新控制器状态，每一帧调用
     */
    void Update() {
        for (size_t i = 0; i < controllers.size(); i++) {
#ifdef _DEBUG
#endif
            controllers[i].Update();
        }
    }

    /**
     * 获取输出的控制指令
     * @return
     */
    std::string GetOutput() {
        std::vector<Instruction> instructions;
        for (auto& c: controllers) {
            instructions.insert(instructions.end(),
                                c.GetInstructionCache().begin(), c.GetInstructionCache().end());
            c.ClearInstructionCache();
        }
        std::string temp;
        for (auto& i: instructions) {
            temp += i.ToString() + '\n';
        }
        return temp;
    }
};

#endif //CODECRAFTSDK_ROBOTCONTROL_HPP
