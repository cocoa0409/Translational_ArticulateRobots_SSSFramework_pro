//
//  main.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//
#include "SSS.hpp"
#include "Painter.hpp"
int main(int argc, const char * argv[]) {
    
    RoundErrorbound=1e-10;
    
    SSS sss;
    //Setup Rod Length
    sss.Setup_RodLength(3, 4);
    
    //Setup fdx,fdy bound & _epsTrans & _epsRot
    sss.Setup_Bound({-10,10}, {-10,10}, 0.01, 0.01);
    
    //Setup Starting Point & destination
    sss.Setup_Goal({-10,-10,0,0},{10,10,180,0});
    
    //Insitu or Movable
    sss.Setup_PathPlanningMode(Movable);
    
    //PreSubdivision rule    ({fdx/fdy?},{fd1,fd2?},GoalDepth)
    sss.Setup_PRE_Subdivision_rule({false, true}, 3);
//    sss.Setup_PRE_Subdivision_rule({true, false}, 6);  //BAD
    sss.Setup_PRE_Subdivision_rule({true, false}, 8);   //GOOD
    
    sss.Build_base_TriangleObstacle(Point_3(11,-11,0.001), Point_3(-11,11,0.002), Point_3(-11,-11,0.003));
    sss.Build_base_TriangleObstacle(Point_3(11,-11,0.001), Point_3(-11,11,0.002), Point_3(11,11,0.003));
    sss.Build_base_TriangleObstacle(Point_3(11,-11,-0.001), Point_3(-11,11,-0.002), Point_3(-11,-11,-0.003));
    sss.Build_base_TriangleObstacle(Point_3(11,-11,-0.001), Point_3(-11,11,-0.002), Point_3(11,11,-0.003));
    sss.printSetupLog();
    
    sss.FindPath();
    sss.BuildRoadMap();
    
    Painter painter;
    painter._baseObstacle_Painter();
    painter.Bound_Painter(sss);
    painter.Path_Painter(sss);
}
