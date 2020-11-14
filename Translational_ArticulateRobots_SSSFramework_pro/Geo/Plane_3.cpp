//
//  Plane_3.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Plane_3.hpp"

Point_3 Plane_3::LineIntersection(const Point_3 & src,const Vector_3 & dir) const{
    if(WhichSide(src)==ON) return src;
    double res=Dot(_NormalVector, dir);
    if(res==0){
        std::cout<<"Wrong Intersection: dir is perpendicular to NormalVector"<<std::endl;
    }
    res= (-Dot(src, _NormalVector)+Dot(_p,_NormalVector)) / res ;
    return src + (dir * res);
}

IntervalT<double> Plane_3::SegmentIntersection(const Segment_3 & s) const{
    PointSetLocation _s_loc=WhichSide(s._s);
    PointSetLocation _d_loc=WhichSide(s._d);
    if( (_s_loc==POS or _s_loc==ON) and (_d_loc==POS or _d_loc==ON))
        return IntervalT<double>(0,1);
    else if(_s_loc==NEG and _d_loc==NEG)
        return IntervalT<double>(INT_MIN,INT_MIN);
    else{
        double t=Dot(_p, _NormalVector)/(Dot(_NormalVector, s._d-s._s));
        assert(t>=0 and t<=1);
        if(_s_loc==NEG)
            return IntervalT<double>(t,1);
        else{
            assert(_d_loc==NEG);
            return IntervalT<double>(0,t);
        }
    }    
}
