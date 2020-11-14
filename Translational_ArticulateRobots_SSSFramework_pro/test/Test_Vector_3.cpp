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
int main(int argc, const char * argv[]) {
    Point_3 A(1,0,0);
    Point_3 B(0,1,0);
    Point_3 C;
    C=A+B;
    std::cout<< C;
    C=A-B;
        std::cout<< C;
    C=Cross(A, B);
        std::cout<< C;
    C=A*2;
        std::cout<< C;
    C=A/5;
    C=C.normalize();
        std::cout<< C;
    std::cout<< Dot(A, B);
}
