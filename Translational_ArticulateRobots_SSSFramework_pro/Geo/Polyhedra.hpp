//
//  Polyhedra.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Polyhedra_hpp
#define Polyhedra_hpp
#include "CONFIG.h"
#include "Vector_3.hpp"
#include "Segment_3.hpp"
#include "Plane_3.hpp"

class Polyhedra{
public:
    std::vector<Point_3> _PointSet;
    std::vector<Segment_3> _SegmentSet;
    std::vector<Plane_3> _PlaneSet;
    
    Polyhedra(){}
    Polyhedra(const size_t pointsize,const size_t ssize,const size_t psize){
        _PointSet.reserve(pointsize);
        _SegmentSet.reserve(ssize);
        _PlaneSet.reserve(psize);
    }
    Polyhedra(const Point_3 & p1,const Point_3 & p2,const Point_3 & p3); //构造三角形
    
    Polyhedra (const Polyhedra & p);//赋值构造函数
    Polyhedra& operator = (const Polyhedra& p);//赋值运算符
    
    ~Polyhedra(){
        _PointSet.clear();
        _SegmentSet.clear();
        _PlaneSet.clear();
    }
    
    void addPoint(const Point_3 &p){
        _PointSet.emplace_back(p);
    }

    void addSegment(const Point_3 & p1,const Point_3 & p2){
        _SegmentSet.emplace_back(p1,p2);
    }
    void addPlane(const Point_3 & p,const Vector_3 & NormalVector){
        _PlaneSet.emplace_back(p,NormalVector);
    }
    
    friend std::ostream & operator<<(std::ostream &out, Polyhedra & v);
};

bool isIntersected(const Polyhedra & P1,const Polyhedra & P2);
bool isIntersected(const Polyhedra & P,const Segment_3 & s);



#endif /* Polyhedra_hpp */
