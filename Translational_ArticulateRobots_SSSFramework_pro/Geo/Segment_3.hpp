//
//  Segment_3.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Segment_3_hpp
#define Segment_3_hpp
#include "CONFIG.h"
#include "Vector_3.hpp"

class Segment_3{
public:
    Point_3 _s;
    Point_3 _d;
    
    Segment_3(){}
    Segment_3(const Point_3 & s,const Point_3 &d):_s(s),_d(d){}
    Segment_3& operator = (const Segment_3& s){
        _s=s._s;
        _d=s._d;
        return *this;
    }
    ~Segment_3(){}
    
    Vector_3 DirectionVector() const{return _d-_s;}
    
};








#endif /* Segment_3_hpp */
