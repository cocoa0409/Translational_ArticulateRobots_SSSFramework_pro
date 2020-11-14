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
int main(int argc, const char * argv[]) {
    RoundErrorbound=1e-10;
    _Rod1len=1;
    _Rod2len=1;
    ParaBox pb({0,1},{0,1},{20,45},{30,80});
    TransObstacle::Build_base_TriangleObstacle({0,0,0}, {2,2,2}, {5,-1,1});
    TransObstacle::Build_base_TriangleObstacle({0,0,0}, {2,2,2}, {-5,-1,1});
    TransObstacle::Build_base_TriangleObstacle({0,0,0}, {1,1,1}, {5,1,0.5});
    TransObstacle::Build_base_TriangleObstacle({0,0,0}, {1,1,1}, {-5,1,0.5});
    for(auto it=TransObstacle::_baseObstacle.begin();it!=TransObstacle::_baseObstacle.end();it++){
        std::cout<<**it;
    }
    TransObstacle t(&pb);
    for(auto it=t._TransObstacle.begin();it!=t._TransObstacle.end();it++){
        std::cout<<**it;
    }
//    Rod_1 r1(&pb);
//    Rod_2 r2(&pb);
    ConfigBox cb(&pb);
}
