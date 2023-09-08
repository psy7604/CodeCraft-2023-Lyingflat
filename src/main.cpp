//#define _DEBUG
#include <iostream>
#include <vector>
#include <memory>
#include <cstring>
#include <cassert>
#include <sstream>
#include <fstream>
#include "RobotControl.hpp"


#ifdef _DEBUG

#include <windows.h>

#endif


using namespace std;


Game game;

GeneralController generalController(game);

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}


bool LoadMap() {
    char line[1025];
    int row = 0;
    bool suc = false;
    while (fgets(line, 1025, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            suc = true;
            break;
        }
        for (int i = 0, n = strlen(line); i < n; i++) {
            switch (line[i]) {
                case '.':
                    break;
                case 'A':
                    game.LoadRobot(0.25 + 0.5 * i, 49.75 - 0.5 * row);
                    break;
                case '1':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 1);
                    break;
                case '2':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 2);
                    break;
                case '3':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 3);
                    break;
                case '4':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 4);
                    break;
                case '5':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 5);
                    break;
                case '6':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 6);
                    break;
                case '7':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 7);
                    break;
                case '8':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 8);
                    break;
                case '9':
                    game.LoadWorktop(0.25 + 0.5 * i, 49.75 - 0.5 * row, 9);
                    break;
                default:
                    break;
            }
        }
        row++;
    };
    return suc;
}

bool LoadFrame() {
    int currentMoney;
    int K;
    int worktopType;
    double worktopPosx, worktopPosy;
    int remainingProductionTime, materialStatus, productionStatus;
    scanf(  // frameID已经被读取
            "%d "
            "%d ", &currentMoney, &K);
    for (int i = 0; i < K; i++) {
        scanf("%d %lf %lf %d %d %d", &worktopType, &worktopPosx, &worktopPosy,
              &remainingProductionTime, &materialStatus, &productionStatus);
        game.RefreshWorktopStatus(i, worktopType, worktopPosx, worktopPosy, remainingProductionTime, materialStatus,
                                  productionStatus);
    }
    int curWorktopID, carryingItemType;
    double timeValueCoefficient, collisionValueCoefficient;
    double palstance, vx, vy, orientation;
    double px, py;
    for (int i = 0; i < 4; i++) {
        scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf", &curWorktopID, &carryingItemType,
              &timeValueCoefficient, &collisionValueCoefficient, &palstance,
              &vx, &vy, &orientation, &px, &py);
        game.RefreshRobotStatus(i, curWorktopID, carryingItemType, timeValueCoefficient, collisionValueCoefficient,
                                palstance, vx, vy, orientation, px, py);
    }
    char temp[1024];
    scanf("%s", temp);

#ifdef _DEBUG

#endif
    return strcmp(temp, "OK") == 0;
}


int main() {
    LoadMap();
    puts("OK");
    fflush(stdout);
    generalController.Init();
    long long frameCount = 0;
    int frameID;
    game.Init();
    while (scanf("%d", &frameID) != EOF) {

#ifdef _DEBUG
#endif

        game.RefreshCurrentFrameID(frameID);
        LoadFrame();
        generalController.Update();
        printf("%d\n", frameID);
        string cache = generalController.GetOutput();
        cout << cache;
        printf("OK\n");

        fflush(stdout);

#ifdef _DEBUG
#endif

        frameCount++;
    }

    return 0;
}
