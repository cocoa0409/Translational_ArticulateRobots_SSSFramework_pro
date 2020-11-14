//
//  TransObstacle.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/6.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "TransObstacle.hpp"
bool IncludeBSTUCKtest;

void TransObstacle::ConstructMinkowskiSumObstacle(const std::shared_ptr<Polyhedra> p,IntervalT<double> & Ix,IntervalT<double> & Iy){
    assert(p->_PointSet.size()==3 and p->_SegmentSet.size()==3 and p->_PlaneSet.size()==5);//保证是三角形
    assert(p->_PointSet[0].z()<p->_PointSet[1].z() and p->_PointSet[1].z()<p->_PointSet[2].z());
    Point_3 p1=p->_PointSet[0];
    Point_3 p2=p->_PointSet[1];
    Point_3 p3=p->_PointSet[2];
    Vector_3 NV=Cross(p3-p2,p2-p1);
    // 把p1/p2/p3投影到YoZ平面上,然后判断p2在p1p3构成的线段,从X轴正向看去的哪一侧;
    TurnDir YoZproj,XoZproj;
    if(NV.x()>=0) YoZproj=LEFT;
    else if(NV.x()<0) YoZproj=RIGHT;
    else std::cout<<"[YoZproj] Wrong Obstacle Triangle for Minkowski sum!"<<std::endl;
    if(NV.y()>=0) XoZproj=LEFT;
    else if(NV.y()<0) XoZproj=RIGHT;
    else std::cout<<"[XoZproj] Wrong Obstacle Triangle for Minkowski sum!"<<std::endl;
    
    std::shared_ptr<Polyhedra> TMS=std::make_shared<Polyhedra>(11,20,10);

    Vector_3 xRyR(Ix.getR(),Iy.getR(),0);
    Vector_3 xLyR(Ix.getL(),Iy.getR(),0);
    Vector_3 xLyL(Ix.getL(),Iy.getL(),0);
    Vector_3 xRyL(Ix.getR(),Iy.getL(),0);
    
    Vector_3 p1_xRyR=p1+xRyR;
    Vector_3 p1_xRyL=p1+xRyL;
    Vector_3 p1_xLyL=p1+xLyL;
    Vector_3 p1_xLyR=p1+xLyR;
    Vector_3 p2_xRyR=p2+xRyR;
    Vector_3 p2_xRyL=p2+xRyL;
    Vector_3 p2_xLyL=p2+xLyL;
    Vector_3 p2_xLyR=p2+xLyR;
    Vector_3 p3_xRyR=p3+xRyR;
    Vector_3 p3_xRyL=p3+xRyL;
    Vector_3 p3_xLyL=p3+xLyL;
    Vector_3 p3_xLyR=p3+xLyR;
    
    if(YoZproj==LEFT and XoZproj==LEFT){
        //三角
        TMS->addSegment(p1_xRyR, p2_xRyR);
        TMS->addSegment(p2_xRyR, p3_xRyR);
        TMS->addSegment(p1_xRyR, p3_xRyR);
        TMS->addPoint(p1_xRyR);
        TMS->addPoint(p2_xRyR);
        TMS->addPoint(p3_xRyR);
        
        TMS->addSegment(p1_xLyL, p2_xLyL);
        TMS->addSegment(p2_xLyL, p3_xLyL);
        TMS->addSegment(p1_xLyL, p3_xLyL);
        TMS->addPoint(p1_xLyL);
        TMS->addPoint(p2_xLyL);
        TMS->addPoint(p3_xLyL);
        
        TMS->addSegment(p1_xRyL, p2_xRyL);
        TMS->addSegment(p2_xRyL, p3_xRyL);
        TMS->addSegment(p1_xRyL, p3_xRyL);
        TMS->addPoint(p1_xRyL);
        TMS->addPoint(p2_xRyL);
        TMS->addPoint(p3_xRyL);
        
        TMS->addSegment(p1_xLyR, p3_xLyR);
        TMS->addPoint(p1_xLyR);
        TMS->addPoint(p3_xLyR);
        
        //平行
        TMS->addSegment(p1_xRyR, p1_xRyL);
        TMS->addSegment(p2_xRyR, p2_xRyL);
        TMS->addSegment(p3_xRyR, p3_xRyL);
        
        TMS->addSegment(p1_xRyL, p1_xLyL);
        TMS->addSegment(p2_xRyL, p2_xLyL);
        TMS->addSegment(p3_xRyL, p3_xLyL);
        
        TMS->addSegment(p1_xLyL, p1_xLyR);
        TMS->addSegment(p3_xLyL, p3_xLyR);
        
        TMS->addSegment(p1_xLyR, p1_xRyR);
        TMS->addSegment(p3_xLyR, p3_xRyR);
        
        //侧面
        TMS->addPlane(p1_xRyR, -NV);
        TMS->addPlane(p1_xLyL, NV);
        
        TMS->addPlane(p1_xRyR, {0,0,1});
        TMS->addPlane(p3_xRyR, {0,0,-1});
        
        TMS->addPlane(p2_xRyL, Cross({1,0,0},p2-p3));
        TMS->addPlane(p2_xRyL, Cross({1,0,0},p1-p2));
        
        TMS->addPlane(p1_xLyR, Cross({1,0,0},p3-p1));
        
        TMS->addPlane(p2_xRyL, Cross({0,1,0},p2-p3));
        TMS->addPlane(p2_xRyL, Cross({0,1,0},p1-p2));
        
        TMS->addPlane(p1_xLyR, Cross({0,1,0},p3-p1));
    }
    else if(YoZproj==LEFT and XoZproj==RIGHT){
        //三角
        TMS->addSegment(p1_xRyL, p2_xRyL);
        TMS->addSegment(p2_xRyL, p3_xRyL);
        TMS->addSegment(p1_xRyL, p3_xRyL);
        TMS->addPoint(p1_xRyL);
        TMS->addPoint(p2_xRyL);
        TMS->addPoint(p3_xRyL);
        
        TMS->addSegment(p1_xLyR, p2_xLyR);
        TMS->addSegment(p2_xLyR, p3_xLyR);
        TMS->addSegment(p1_xLyR, p3_xLyR);
        TMS->addPoint(p1_xLyR);
        TMS->addPoint(p2_xLyR);
        TMS->addPoint(p3_xLyR);
        
        TMS->addSegment(p1_xLyL, p2_xLyL);
        TMS->addSegment(p2_xLyL, p3_xLyL);
        TMS->addSegment(p1_xLyL, p3_xLyL);
        TMS->addPoint(p1_xLyL);
        TMS->addPoint(p2_xLyL);
        TMS->addPoint(p3_xLyL);
        
        TMS->addSegment(p1_xRyR, p3_xRyR);
        TMS->addPoint(p1_xRyR);
        TMS->addPoint(p3_xRyR);
        
        //平行
        TMS->addSegment(p1_xRyL, p1_xLyL);
        TMS->addSegment(p2_xRyL, p2_xLyL);
        TMS->addSegment(p3_xRyL, p3_xLyL);
        
        TMS->addSegment(p1_xLyL, p1_xLyR);
        TMS->addSegment(p2_xLyL, p2_xLyR);
        TMS->addSegment(p3_xLyL, p3_xLyR);
        
        TMS->addSegment(p1_xLyR, p1_xRyR);
        TMS->addSegment(p3_xLyR, p3_xRyR);
        
        TMS->addSegment(p1_xRyR, p1_xRyL);
        TMS->addSegment(p3_xRyR, p3_xRyL);
        
        //侧面
        TMS->addPlane(p1_xRyL, -NV);
        TMS->addPlane(p1_xLyR, NV);
        
        TMS->addPlane(p1_xRyL, {0,0,1});
        TMS->addPlane(p3_xRyL, {0,0,-1});
        
        
        TMS->addPlane(p2_xLyL, Cross({1,0,0},p2-p3));
        TMS->addPlane(p2_xLyL, Cross({1,0,0},p1-p2));
        
        TMS->addPlane(p1_xRyR, Cross({1,0,0},p3-p1));
        
        TMS->addPlane(p2_xLyL, -Cross({0,1,0},p2-p3));
        TMS->addPlane(p2_xLyL, -Cross({0,1,0},p1-p2));
        
        TMS->addPlane(p1_xRyR, -Cross({0,1,0},p3-p1));
    }
    else if(YoZproj==RIGHT and XoZproj==RIGHT){
        //三角
        TMS->addSegment(p1_xLyL, p2_xLyL);
        TMS->addSegment(p2_xLyL, p3_xLyL);
        TMS->addSegment(p1_xLyL, p3_xLyL);
        TMS->addPoint(p1_xLyL);
        TMS->addPoint(p2_xLyL);
        TMS->addPoint(p3_xLyL);
        
        TMS->addSegment(p1_xRyR, p2_xRyR);
        TMS->addSegment(p2_xRyR, p3_xRyR);
        TMS->addSegment(p1_xRyR, p3_xRyR);
        TMS->addPoint(p1_xRyR);
        TMS->addPoint(p2_xRyR);
        TMS->addPoint(p3_xRyR);
        
        TMS->addSegment(p1_xLyR, p2_xLyR);
        TMS->addSegment(p2_xLyR, p3_xLyR);
        TMS->addSegment(p1_xLyR, p3_xLyR);
        TMS->addPoint(p1_xLyR);
        TMS->addPoint(p2_xLyR);
        TMS->addPoint(p3_xLyR);
        
        TMS->addSegment(p1_xRyL, p3_xRyL);
        TMS->addPoint(p1_xRyL);
        TMS->addPoint(p3_xRyL);
        
        //平行
        TMS->addSegment(p1_xLyL, p1_xLyR);
        TMS->addSegment(p2_xLyL, p2_xLyR);
        TMS->addSegment(p3_xLyL, p3_xLyR);
        
        TMS->addSegment(p1_xLyR, p1_xRyR);
        TMS->addSegment(p2_xLyR, p2_xRyR);
        TMS->addSegment(p3_xLyR, p3_xRyR);
        
        TMS->addSegment(p1_xRyR, p1_xRyL);
        TMS->addSegment(p3_xRyR, p3_xRyL);
        
        TMS->addSegment(p1_xRyL, p1_xLyL);
        TMS->addSegment(p3_xRyL, p3_xLyL);
        
        //侧面
        TMS->addPlane(p1_xLyL, -NV);
        TMS->addPlane(p1_xRyR, NV);
        
        TMS->addPlane(p1_xLyL, {0,0,1});
        TMS->addPlane(p3_xLyL, {0,0,-1});

        TMS->addPlane(p2_xLyR, -Cross({1,0,0},p2-p3));
        TMS->addPlane(p2_xLyR, -Cross({1,0,0},p1-p2));
        
        TMS->addPlane(p1_xRyL, -Cross({1,0,0},p3-p1));
        
        TMS->addPlane(p2_xLyR, -Cross({0,1,0},p2-p3));
        TMS->addPlane(p2_xLyR, -Cross({0,1,0},p1-p2));
        
        TMS->addPlane(p1_xRyL, -Cross({0,1,0},p3-p1));
    }
    else if(YoZproj==RIGHT and XoZproj==LEFT){
        //三角
        TMS->addSegment(p1_xLyR, p2_xLyR);
        TMS->addSegment(p2_xLyR, p3_xLyR);
        TMS->addSegment(p1_xLyR, p3_xLyR);
        TMS->addPoint(p1_xLyR);
        TMS->addPoint(p2_xLyR);
        TMS->addPoint(p3_xLyR);
        
        TMS->addSegment(p1_xRyL, p2_xRyL);
        TMS->addSegment(p2_xRyL, p3_xRyL);
        TMS->addSegment(p1_xRyL, p3_xRyL);
        TMS->addPoint(p1_xRyL);
        TMS->addPoint(p2_xRyL);
        TMS->addPoint(p3_xRyL);
        
        TMS->addSegment(p1_xRyR, p2_xRyR);
        TMS->addSegment(p2_xRyR, p3_xRyR);
        TMS->addSegment(p1_xRyR, p3_xRyR);
        TMS->addPoint(p1_xRyR);
        TMS->addPoint(p2_xRyR);
        TMS->addPoint(p3_xRyR);
        
        TMS->addSegment(p1_xLyL, p3_xLyL);
        TMS->addPoint(p1_xLyL);
        TMS->addPoint(p3_xLyL);
        
        //平行
        TMS->addSegment(p1_xLyR, p1_xRyR);
        TMS->addSegment(p2_xLyR, p2_xRyR);
        TMS->addSegment(p3_xLyR, p3_xRyR);
        
        TMS->addSegment(p1_xRyR, p1_xRyL);
        TMS->addSegment(p2_xRyR, p2_xRyL);
        TMS->addSegment(p3_xRyR, p3_xRyL);
        
        TMS->addSegment(p1_xRyL, p1_xLyL);
        TMS->addSegment(p3_xRyL, p3_xLyL);
        
        TMS->addSegment(p1_xLyL, p1_xLyR);
        TMS->addSegment(p3_xLyL, p3_xLyR);
        
        //侧面
        TMS->addPlane(p1_xLyR, -NV);
        TMS->addPlane(p1_xRyL, NV);
        
        TMS->addPlane(p1_xLyR, {0,0,1});
        TMS->addPlane(p3_xLyR, {0,0,-1});
        
        
        TMS->addPlane(p2_xRyR, -Cross({1,0,0},p2-p3));
        TMS->addPlane(p2_xRyR, -Cross({1,0,0},p1-p2));
        
        TMS->addPlane(p1_xLyL, -Cross({1,0,0},p3-p1));
        
        TMS->addPlane(p2_xRyR, Cross({0,1,0},p2-p3));
        TMS->addPlane(p2_xRyR, Cross({0,1,0},p1-p2));
        
        TMS->addPlane(p1_xLyL, Cross({0,1,0},p3-p1));
    }
    else std::cout<<"[YoZproj][XoZproj] Wrong Obstacle Triangle for Minkowski sum!"<<std::endl;
    
//    for(auto it=TMS->_PlaneSet.begin();it!=TMS->_PlaneSet.end();it++){
//        assert((*it).WhichSide(p1_xRyR)!=NEG);
//        assert((*it).WhichSide(p1_xRyL)!=NEG);
//        assert((*it).WhichSide(p1_xLyL)!=NEG);
//        assert((*it).WhichSide(p1_xLyR)!=NEG);
//        assert((*it).WhichSide(p2_xRyR)!=NEG);
//        assert((*it).WhichSide(p2_xRyL)!=NEG);
//        assert((*it).WhichSide(p2_xLyL)!=NEG);
//        assert((*it).WhichSide(p2_xLyR)!=NEG);
//        assert((*it).WhichSide(p3_xRyR)!=NEG);
//        assert((*it).WhichSide(p3_xRyL)!=NEG);
//        assert((*it).WhichSide(p3_xLyL)!=NEG);
//        assert((*it).WhichSide(p3_xLyR)!=NEG);
//    }
    
    _TransObstacle.push_back(TMS);
    
    
    if(IncludeBSTUCKtest){
        std::vector<std::shared_ptr<Triangle_3>> res;
        res.push_back(std::make_shared<Triangle_3>(p1+xRyR,p2+xRyR,p3+xRyR));
        res.push_back(std::make_shared<Triangle_3>(p1+xRyL,p2+xRyL,p3+xRyL));
        res.push_back(std::make_shared<Triangle_3>(p1+xLyR,p2+xLyR,p3+xLyR));
        res.push_back(std::make_shared<Triangle_3>(p1+xLyL,p2+xLyL,p3+xLyL));
        _StuckObstacle.push_back(res);
    }
}



std::vector<std::shared_ptr<Polyhedra>> TransObstacle::_baseObstacle(0);

void TransObstacle::Build_base_TriangleObstacle(const Point_3 & p1,const Point_3 & p2,const Point_3 & p3){
    _baseObstacle.emplace_back(std::make_shared<Polyhedra>(p1,p2,p3));//构造三角形
    ParaBox::_obs_size++;
}

TransObstacle::TransObstacle(ParaBox * P){
    /*
     优化部分
     
     
     
     
     */
    //根据negative IxIy构造出 空间三角形 与 XoY平面长方形的minkowski和 多面体
    
    IntervalT<double> negIx(-(P->fdx_getR()),-(P->fdx_getL()));
    IntervalT<double> negIy(-(P->fdy_getR()),-(P->fdy_getL()));
    for(auto it=_baseObstacle.begin();it!=_baseObstacle.end();it++){
        ConstructMinkowskiSumObstacle(*it, negIx, negIy);
    }
    
    
}
