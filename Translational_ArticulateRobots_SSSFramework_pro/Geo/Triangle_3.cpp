//
//  Triangle_3.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/6/11.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Triangle_3.hpp"
bool isIntersected(const Triangle_3 & T,const Segment_3 & s){
    double d=Dot((s._s-s._d), T._NormalVector);
    if(d==0) return false; //Segment平行与Triangle时，不考虑相交问题
    Vector_3 e = Cross(s._s-s._d,s._s-T._p1);
    
    double t = Dot(s._s-T._p1,T._NormalVector)/d;
    if(t<0 or t>1) return false;
    
    double v = Dot(T._p3-T._p1,e)/d;
    if(v<0 or v>1) return false;
    
    double w = -Dot(T._p2-T._p1,e)/d;
    if(w<0 or w+v>1) return false;
    
    return true;
}
