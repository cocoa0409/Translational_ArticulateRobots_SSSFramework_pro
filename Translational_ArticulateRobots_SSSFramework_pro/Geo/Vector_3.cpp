//
//  Vector_3.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Vector_3.hpp"
double RoundErrorbound;

////三维向量点积
//double Dot (const Vector_3& l, const Vector_3& r)
//{
//  return l.x()*r.x() + l.y()*r.y() + l.z()*r.z();
//}
 
//// 三维向量叉积
//Vector_3 Cross (const Vector_3& l, const Vector_3& r)
//{
//  return Vector_3(
//    l.y()*r.z() - l.z()*r.y(),
//    l.z()*r.x() - l.x()*r.z(),
//    l.x()*r.y() - l.y()*r.x() );
//}
 
// 三维向量混合积
double BlendProduct (const Vector_3& l, const Vector_3& m, const Vector_3& r)
{
  return Dot(Cross(l, m), r);
}

PointSetLocation WhichSide(const std::vector<Point_3> & S, const Vector_3 & D, const Point_3 & P){
    int positive=0;
    int negative=0;
    int on=0;
    for(auto it=S.begin();it!=S.end();it++){
        double t=Dot(D, (*it)-P);
        if(t>RoundErrorbound) positive++;
        else if(t<-RoundErrorbound) negative++;
        else on++;
        if(positive && negative) return NotSame;
    }
    if(on!=0){
        if(positive==0 and negative==0) return ON;
        return positive?POS_ON:ON_NEG;
    }
    return positive?POS:NEG;
}


std::ostream & operator<<(std::ostream &out, Vector_3 & v){
    out <<"("<<v.x()<<","<<v.y()<<","<<v.z()<<")";
    return out;
}
