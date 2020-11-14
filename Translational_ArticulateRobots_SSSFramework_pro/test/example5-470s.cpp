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
    sss.Setup_PRE_Subdivision_rule({false, true}, 5);
    sss.Setup_PRE_Subdivision_rule({true, false}, 11);   //GOODtop
    
    
    sss.Build_base_TriangleObstacle(Point_3(15,-0.00001,4), Point_3(15,0.00001,-4.002), Point_3(-8,0,3.9));
    sss.Build_base_TriangleObstacle(Point_3(-8,0,3.9), Point_3(-8,0.0002,-4.1), Point_3(15,0.00001,-4.002));
    
    sss.Build_base_TriangleObstacle(Point_3(15,5.00001,4), Point_3(15,5.00001,-4.002), Point_3(-8,5,3.9));
    sss.Build_base_TriangleObstacle(Point_3(-8,5,3.9), Point_3(-8,5.0002,-4.1), Point_3(15,5.00001,-4.002));

    sss.Build_base_TriangleObstacle(Point_3(15,-5.00001,4), Point_3(15,-5.00001,-4.002), Point_3(-8,-5,3.9));
    sss.Build_base_TriangleObstacle(Point_3(-8,-5,3.9), Point_3(-8,-5.0002,-4.1), Point_3(15,-5.00001,-4.002));
    
    sss.Build_base_TriangleObstacle(Point_3(-15,2.50001,4), Point_3(-15,2.50001,-4.002), Point_3(8,2.5,3.9));
    sss.Build_base_TriangleObstacle(Point_3(8,2.5,3.9), Point_3(8,2.50002,-4.1), Point_3(-15,2.50001,-4.002));

    sss.Build_base_TriangleObstacle(Point_3(-15,-2.50001,4), Point_3(-15,-2.50001,-4.002), Point_3(8,-2.5,3.9));
    sss.Build_base_TriangleObstacle(Point_3(8,-2.5,3.9), Point_3(8,-2.5002,-4.1), Point_3(-15,-2.50001,-4.002));

//    sss.Build_base_TriangleObstacle(Point_3(-15,-7.500001,4), Point_3(-15,-7.50001,-4.002), Point_3(8,-7.5,3.9));
//    sss.Build_base_TriangleObstacle(Point_3(8,-7.5,3.9), Point_3(8,-7.50002,-4.1), Point_3(-15,-7.500001,-4.002));
//
//    sss.Build_base_TriangleObstacle(Point_3(-15,7.500001,4), Point_3(-15,7.500001,-4.002), Point_3(8,7.5,3.9));
//    sss.Build_base_TriangleObstacle(Point_3(8,7.5,3.9), Point_3(8,7.50002,-4.1), Point_3(-15,7.500001,-4.002));
    
    Painter painter;
    painter._baseObstacle_Painter();
    painter.Bound_Painter(sss);
    sss.printSetupLog();
    
    sss.FindPath();
    sss.BuildRoadMap();
    
    painter._baseObstacle_Painter();
    painter.Bound_Painter(sss);
    painter.Path_Painter(sss);
}
