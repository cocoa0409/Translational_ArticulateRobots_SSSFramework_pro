//
//  Plane_3.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Plane_3_hpp
#define Plane_3_hpp
#include "CONFIG.h"
#include "Vector_3.hpp"
#include "Segment_3.hpp"
#include "IntervalT.h"

class Plane_3{
public:
    Point_3 _p;
    Vector_3 _NormalVector;
    
    Plane_3(){};
    Plane_3(const Point_3 & p1,const Point_3 & p2,const Point_3 & p3){
        _p=p1;
        _NormalVector=Cross(p2-p1, p3-p2);
        _NormalVector.normalize();
    }
    Plane_3(const Point_3 & p,const Vector_3 & NV){
        _p=p;
        _NormalVector=NV;
        _NormalVector.normalize();
    }
    Plane_3 & operator = (const Plane_3 & p){
        _p=p._p;
        _NormalVector=p._NormalVector;
        return *this;
    }
    
    
    PointSetLocation WhichSide(const Point_3 & p) const{
        double res=Dot(p-_p, _NormalVector);
        if(abs(res)<=RoundErrorbound) return ON;
        else if(res>RoundErrorbound) return POS;
        else return NEG;
    }
    
    Point_3 LineIntersection(const Point_3 & src,const Vector_3 & dir) const;
    
    IntervalT<double> SegmentIntersection(const Segment_3 & s) const;
};

#endif /* Plane_3_hpp */
