//
//  CONFIG.h
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef CONFIG_h
#define CONFIG_h

#include <iostream>
#include <fstream>
#include <math.h>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <float.h>
class Vector_3;
typedef Vector_3 Point_3;


enum ConfigBoxStatus { BFREE, BMIXED, BUNKNOWN ,BSTUCK};
enum FD2Status {N180toN90,N90toP0,P0toP90,P90toP180,INI};
enum SPLITMODE{TransRot,Transonly,Rotonly,NOOP};
enum PathPlanningMode {Insitu,Movable};
enum TurnDir{LEFT,RIGHT};

enum PointSetLocation{POS,POS_ON,ON,ON_NEG,NEG,NotSame};

extern double RoundErrorbound;
extern double _Rod1len;
extern double _Rod2len;
extern bool IncludeBSTUCKtest;

#endif /* CONFIG_h */
