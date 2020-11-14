//
//  main.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include <iostream>
#include <unordered_map>
#include <vector>
#include "Box/ParaBox.hpp"
#include "Geo/Vector_3.hpp"
#include "CONFIG.h"
#include "Rod_1.hpp"
#include "Rod_2.hpp"
#include "hashmapUtil.h"
#include "TransObstacle.hpp"
#include "ConfigBox.hpp"
#include "SSS.hpp"
int main(int argc, const char * argv[]) {
    
    RoundErrorbound=1e-10;
    
    SSS sss;
    //Setup Rod Length
    sss.Setup_RodLength(3, 4);
    
    //Setup fdx,fdy bound & _epsTrans & _epsRot
    sss.Setup_Bound({-20,20}, {-20,20}, 0.01, 0.01);
    
    //Setup Starting Point & destination
    sss.Setup_Goal({-10,-10,0,0},{10,10,-180,180});
    
    //Insitu or Movable
    sss.Setup_PathPlanningMode(Movable);
    
    //PreSubdivision rule    ({fdx/fdy?},{fd1,fd2?},GoalDepth)
    sss.Setup_PRE_Subdivision_rule({false, true}, 3);
    
    sss.Build_base_TriangleObstacle(Point_3(5,-8,1), Point_3(-5,-8,0.9), Point_3(0,-8,5));
    sss.Build_base_TriangleObstacle(Point_3(5,0,1), Point_3(-5,0,0.9), Point_3(0,0,5));
    sss.Build_base_TriangleObstacle(Point_3(5,0,-1), Point_3(-5,0,-0.9), Point_3(0,0,-5));
    sss.Build_base_TriangleObstacle(Point_3(-4,0,1), Point_3(5,5,0.9), Point_3(5,-5,1.1));
    sss.Build_base_TriangleObstacle(Point_3(0,1,2), Point_3(1,2,3), Point_3(3,-2,1));
    sss.Build_base_TriangleObstacle(Point_3(-0,-1,-2), Point_3(-1,-2,3), Point_3(3,-2,1));
    sss.Build_base_TriangleObstacle(Point_3(0,-1,2), Point_3(1,-2,3), Point_3(3,-2,-1));
    sss.Build_base_TriangleObstacle(Point_3(0,1,-2), Point_3(-1,2,-3), Point_3(-3,-2,-1));
    sss.Build_base_TriangleObstacle(Point_3(5,9,-1), Point_3(1,2,6), Point_3(-2,3,0));
    sss.Build_base_TriangleObstacle(Point_3(8,10,-1), Point_3(8,10,1), Point_3(-2,3,-4));
    sss.printSetupLog();
    
    sss.FindPath();
    
    sss.BuildRoadMap();
    
    Painter painter;
    painter._baseObstacle_Painter();
    painter.Bound_Painter(sss);
    painter.Path_Painter(sss);
    
}
