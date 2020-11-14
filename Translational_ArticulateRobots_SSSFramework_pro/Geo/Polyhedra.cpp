//
//  Polyhedra.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Polyhedra.hpp"

Polyhedra::Polyhedra (const Polyhedra & p){//赋值构造函数
    _PointSet.reserve(p._PointSet.size());
    _SegmentSet.reserve(p._SegmentSet.size());
    _PlaneSet.reserve(p._PlaneSet.size());
    std::copy(p._PointSet.begin(), p._PointSet.end(), std::back_inserter(_PointSet));
    std::copy(p._SegmentSet.begin(), p._SegmentSet.end(), std::back_inserter(_SegmentSet));
    std::copy(p._PlaneSet.begin(), p._PlaneSet.end(), std::back_inserter(_PlaneSet));
}

Polyhedra& Polyhedra::operator = (const Polyhedra& p){//赋值运算符
    _PointSet.clear();
    _PointSet.reserve(p._PointSet.size());
    _SegmentSet.clear();
    _SegmentSet.reserve(p._SegmentSet.size());
    _PlaneSet.clear();
    _PlaneSet.reserve(p._PlaneSet.size());
    std::copy(p._PointSet.begin(), p._PointSet.end(), std::back_inserter(_PointSet));
    std::copy(p._SegmentSet.begin(), p._SegmentSet.end(), std::back_inserter(_SegmentSet));
    std::copy(p._PlaneSet.begin(), p._PlaneSet.end(), std::back_inserter(_PlaneSet));
    return *this;
}


bool minZfirst(Point_3 i,Point_3 j) { return (i.z()<j.z()); }

Polyhedra::Polyhedra(const Point_3 & p1,const Point_3 & p2,const Point_3 & p3){
    _PointSet.reserve(3);
    _SegmentSet.reserve(3);
    _PlaneSet.reserve(5);
    
    addPoint(p1);
    addPoint(p2);
    addPoint(p3);
    addSegment(p1, p2);
    addSegment(p2, p3);
    addSegment(p1, p3);
    Vector_3 NV=Cross(p2-p1,p3-p2);
    addPlane(p2, NV);
    addPlane(p2,-NV);
    addPlane(p2, Cross(NV, p2-p1));
    addPlane(p1, Cross(NV, p1-p3));
    addPlane(p3, Cross(NV, p3-p2));
    
    Point_3 testPoint=(p1+p2+p3)/3;
    
    for(auto it=_PlaneSet.begin();it!=_PlaneSet.end();it++){
        assert((*it).WhichSide(testPoint)!=NEG);
    }
//    assert(abs(NV.x())>RoundErrorbound and abs(NV.y())>RoundErrorbound);//要求构造出的三角形满足
//    assert(_PointSet[0].z()!=_PointSet[1].z() and _PointSet[0].z()!=_PointSet[2].z() and _PointSet[1].z()!=_PointSet[2].z());
    
    std::sort(_PointSet.begin(), _PointSet.end(), minZfirst);//按Z坐标排序
    assert(_PointSet[0].z()<=_PointSet[1].z() and _PointSet[1].z()<=_PointSet[2].z());
}


std::ostream & operator<<(std::ostream &out, Polyhedra & v){
    out <<"[Polyhedra] "<<v._PointSet.size()<<" points,"<<v._SegmentSet.size()<<" segments,"<<v._PlaneSet.size()<<" planes"<<std::endl;
    return out;
}

//enum PointSetLocation{POS,POS_ON,ON,ON_NEG,NEG,NotSame};
bool PointSetLocationComplementary(const PointSetLocation & psl0,const PointSetLocation & psl1){
    if(psl0==POS){
        if(psl1==ON or psl1==ON_NEG or psl1==NEG) return true;
        else return false;
    }
    else if(psl0==POS_ON){
        if(psl1==NEG) return true;
        else return false;
    }
    else if(psl0==ON){
        if(psl1==POS or psl1==NEG) return true;
        else return false;
    }
    else if(psl0==ON_NEG){
        if(psl1==POS) return true;
        else return false;
    }
    else if(psl0==NEG){
        if(psl1==ON or psl1==POS_ON or psl1==POS) return true;
        else return false;
    }
    else{
        std::cout<<"WRONG PointSetLocation Value;"<<std::endl;
    }
    return false;
}

bool isIntersected(const Polyhedra & C0,const Polyhedra & C1){
    PointSetLocation psl;
    for(auto it=C0._PlaneSet.begin();it!=C0._PlaneSet.end();it++){
        psl=WhichSide(C1._PointSet,it->_NormalVector, it->_p);
        if(psl==NEG){//如果C1的点全在C0的某个面的Negative侧
            return false;
        }
    }
    for(auto it=C1._PlaneSet.begin();it!=C1._PlaneSet.end();it++){
        psl=WhichSide(C0._PointSet,it->_NormalVector, it->_p);
        if(psl==NEG){//如果C0的点全在C1的某个面的Negative侧
            return false;
        }
    }
    
    Vector_3 D;
    PointSetLocation psl0;
    PointSetLocation psl1;
    for(auto it0=C0._SegmentSet.begin();it0!=C0._SegmentSet.end();it0++){
        for(auto it1=C1._SegmentSet.begin();it1!=C1._SegmentSet.end();it1++){
            D=Cross(it0->DirectionVector(), it1->DirectionVector());
            
            if(D.iszero()) continue;
            
            D.normalize();
            
            psl0=WhichSide(C0._PointSet, D, it1->_s);
            if(psl0==NotSame) continue;
            
            psl1=WhichSide(C1._PointSet, D, it1->_s);
            if(psl1==NotSame) continue;
            
            if(PointSetLocationComplementary(psl0, psl1))
                return false;
        }
    }
    return true;
}


bool isIntersected(const Polyhedra & P,const Segment_3 & s){
    IntervalT<double> ref(0,1);
    IntervalT<double> tmp;
    for(auto it=P._PlaneSet.begin();it!=P._PlaneSet.end();it++){
        tmp=(*it).SegmentIntersection(s);
        if(weakOverlap(ref, tmp)==true){
            ref=Intersect(ref, tmp);
        }
        else{
            return false;
        }
    }
    return true;
}
