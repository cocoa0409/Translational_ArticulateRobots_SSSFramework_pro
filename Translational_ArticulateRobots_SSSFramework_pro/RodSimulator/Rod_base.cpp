//
//  Rod_base.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Rod_base.hpp"
double _Rod1len;
double _Rod2len;
double sinA(double x){
    return (x==-180 or x==180 or x==0)?0:sin(x*M_PI/180);
}
double cosA(double x){
    return (x==90 or x==-90)?0:cos(x*M_PI/180);
}
