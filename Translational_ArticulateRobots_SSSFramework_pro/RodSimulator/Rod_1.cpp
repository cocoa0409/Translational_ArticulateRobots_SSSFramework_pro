//
//  Rod_1.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Rod_1.hpp"

Rod_1::Rod_1(ParaBox * P){
    /*
     优化部分
     
     
     
     
     */
    
    //构造P1的凸拟合
    F1=Point_3(0,0,0);
    F2=Point_3(-sinA(P->fd1_getL())*_Rod1len,-cosA(P->fd1_getL())*_Rod1len,0);//P1left
    F4=Point_3(-sinA(P->fd1_getR())*_Rod1len,-cosA(P->fd1_getR())*_Rod1len,0);//P1right
    F3=F2+F4;
    F3.normalize();
    F3*=_Rod1len/cosA(P->fd1_width()/2);
    build();
//    for(auto it=_CEMRot->_PlaneSet.begin();it!=_CEMRot->_PlaneSet.end();it++){
//        assert((*it).WhichSide(F1)!=NEG);
//        assert((*it).WhichSide(F2)!=NEG);
//        assert((*it).WhichSide(F3)!=NEG);
//        assert((*it).WhichSide(F4)!=NEG);
//    }
    
//    std::cout<<"Rod_1 done! "<<*P;
//    std::cout<<"Rod_1 :"<<*_CEMRot;
}



void Rod_1::build(){
    _CEMRot=std::make_shared<Polyhedra>(4,4,6);
    _CEMRot->addPoint(F1);
    _CEMRot->addPoint(F2);
    _CEMRot->addPoint(F3);
    _CEMRot->addPoint(F4);
    _CEMRot->addSegment(F1, F2);
    _CEMRot->addSegment(F1, F4);
    _CEMRot->addSegment(F2, F3);
    _CEMRot->addSegment(F3, F4);
    _CEMRot->addPlane(F1, {0,0,1});
    _CEMRot->addPlane(F1, {0,0,-1});
    _CEMRot->addPlane(F1, F3-F2);
    _CEMRot->addPlane(F1, F3-F4);
    _CEMRot->addPlane(F3, F1-F2);
    _CEMRot->addPlane(F3, F1-F4);
}

