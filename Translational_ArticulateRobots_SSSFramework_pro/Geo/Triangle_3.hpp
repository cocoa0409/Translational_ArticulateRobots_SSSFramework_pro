//
//  Triangle_3.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/6/11.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Triangle_3_hpp
#define Triangle_3_hpp

#include "CONFIG.h"
#include "Vector_3.hpp"
#include "Segment_3.hpp"

class Triangle_3{
public:
    Point_3 _p1,_p2,_p3;
    Vector_3 _NormalVector;
    
    Triangle_3(){}
    Triangle_3(const Point_3 & p1,const Point_3 & p2,const Point_3 & p3){ //构造三角形
        _p1=p1;
        _p2=p2;
        _p3=p3;
        _NormalVector=Cross(_p2-_p1, _p3-_p1);
    }
    ~Triangle_3(){}
};

bool isIntersected(const Triangle_3 & T,const Segment_3 & s);


#endif /* Triangle_3_hpp */
