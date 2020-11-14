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
    
    //Setup enable BSTUCK test
    sss.Setup_IncludeBSTUCKtest(false);
    
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

    sss.Build_base_TriangleObstacle(Point_3(-15,-9.00001,4), Point_3(-15,-9.0001,-4.002), Point_3(8,-9,3.9));
    sss.Build_base_TriangleObstacle(Point_3(8,-9,3.9), Point_3(8,-9.0002,-4.1), Point_3(-15,-9.00001,-4.002));

    sss.Build_base_TriangleObstacle(Point_3(0,-14.00001,1), Point_3(0,-6.00001,1.002), Point_3(0,-10,5));
    sss.Build_base_TriangleObstacle(Point_3(0,-14.00001,-1), Point_3(0,-6.0000,-1.002), Point_3(0,-10,-5));
    sss.Build_base_TriangleObstacle(Point_3(7,-7,4), Point_3(7.0123,-7.0002,-4.1), Point_3(8,-7.0001,0.002));
    
//    sss.Build_base_TriangleObstacle(Point_3(7,-11,4), Point_3(7.0123,-11.0002,-4.1), Point_3(8,-11.0001,0.002));
    
    sss.Build_base_TriangleObstacle(Point_3(-7,-7,4), Point_3(-7.0123,-7.0002,-4.1), Point_3(-8,-7.0001,0.002));
    
    sss.Build_base_TriangleObstacle(Point_3(-7,-11,4), Point_3(-7.0123,-11.0002,-4.1), Point_3(-8,-11.0001,0.002));
    
    sss.Build_base_TriangleObstacle(Point_3(15,8.00001,4), Point_3(15,8.00001,-4.002), Point_3(1,8,3.9));
    sss.Build_base_TriangleObstacle(Point_3(1,8,3.9), Point_3(1,8.0002,-4.1), Point_3(15,8.00001,-4.002));
    
    sss.Build_base_TriangleObstacle(Point_3(-15,8.00001,4), Point_3(-15,8.00001,-4.002), Point_3(-1,8,3.9));
    sss.Build_base_TriangleObstacle(Point_3(-1,8,3.9), Point_3(-1,8.0002,-4.1), Point_3(-15,8.00001,-4.002));
    
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
