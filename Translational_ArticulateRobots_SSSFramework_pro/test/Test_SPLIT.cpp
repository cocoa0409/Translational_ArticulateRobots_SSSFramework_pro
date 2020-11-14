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
    sss.Setup_RodLength(1, 1);
    sss.Setup_Bound({-5,7}, {9,20}, 0.1, 1);
    sss.Setup_Goal({-2,18,30,140}, {6,10,-167,3});
    sss.Setup_PathPlanningMode(Movable);
    
    sss.Build_base_TriangleObstacle({0,0,0}, {2,2,2}, {5,-1,1});
    sss.Build_base_TriangleObstacle({0,0,0}, {2,2,2}, {-5,-1,1});
    sss.Build_base_TriangleObstacle({0,0,0}, {1,1,1}, {5,1,0.5});
    sss.Build_base_TriangleObstacle({0,0,0}, {1,1,1}, {-5,1,0.5});
    
    sss.printSetupLog();
    
    ParaBox pb({0,6},{2,18},{20,45},{2,78});
    pb.SPLIT(sss._epsTrans, sss._epsRot);
    ConfigBox * cb;
    for(auto it=pb._childrens.begin();it!=pb._childrens.end();it++){
        cb=new ConfigBox(*it);
        std::cout<<"add ConfigBox of "<<**it;
        std::cout<<"--Rod_1Repo.size()="<<ConfigBox::Rod_1Repo.size()<<";  Rod_2Repo.size()="<<ConfigBox::Rod_2Repo.size()<<";  TransRepo.size()="<<ConfigBox::TransRepo.size()<<std::endl;
    }
    for(auto it=pb._childrens.begin();it!=pb._childrens.end();it++){
        delete (*it)->_ConfigBox ;
        std::cout<<"--Rod_1Repo.size()="<<ConfigBox::Rod_1Repo.size()<<";  Rod_2Repo.size()="<<ConfigBox::Rod_2Repo.size()<<";  TransRepo.size()="<<ConfigBox::TransRepo.size()<<std::endl;
    }
    
    
    sss.clearRod_1Repo();
    sss.clearRod_2Repo();
    sss.clearTransRepo();

    
}
