//
//  Rod_2.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Rod_2.hpp"
Rod_2::Rod_2(ParaBox * P){
    /*
     优化部分
     
     
     
     
     */
    
    
    //根据P2的不同区间取值初始化特征
    double sinA_P1L=sinA(P->fd1_getL()),sinA_P1R=sinA(P->fd1_getR()),cosA_P1L=cosA(P->fd1_getL()),cosA_P1R=cosA(P->fd1_getR());
    double sinA_P2L=sinA(P->fd2_getL()),sinA_P2R=sinA(P->fd2_getR()),cosA_P2L=cosA(P->fd2_getL()),cosA_P2R=cosA(P->fd2_getR());
    Point_3 front_down,front_mid,front_top,back_down,back_mid,back_top;
    if(P->_FD2Status == P0toP90 or P->_FD2Status==P90toP180){
        front_down = Point_3(- sinA_P1L*_Rod1len,- cosA_P1L*_Rod1len,0);//P1left
        back_down = Point_3(- sinA_P1R*_Rod1len,- cosA_P1R*_Rod1len,0);//P1right
        if(P->_FD2Status == P0toP90){
            front_mid = Point_3(cosA_P1L*sinA_P2R*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2R*_Rod2len- cosA_P1L*_Rod1len,cosA_P2R*_Rod2len);//P1left P2right
            front_top = Point_3(cosA_P1L*sinA_P2L*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2L*_Rod2len- cosA_P1L*_Rod1len,cosA_P2L*_Rod2len);//P1left P2left
            back_mid = Point_3(cosA_P1R*sinA_P2R*_Rod2len - sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2R*_Rod2len- cosA_P1R*_Rod1len,cosA_P2R*_Rod2len);//P1right P2right
            back_top = Point_3(cosA_P1R*sinA_P2L*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2L*_Rod2len- cosA_P1R*_Rod1len,cosA_P2L*_Rod2len);//P1right P2left
        }
        else{
            assert(P->_FD2Status==P90toP180);
            front_mid = Point_3(cosA_P1L*sinA_P2L*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2L*_Rod2len- cosA_P1L*_Rod1len,cosA_P2L*_Rod2len);//P1left P2left
            front_top = Point_3(cosA_P1L*sinA_P2R*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2R*_Rod2len- cosA_P1L*_Rod1len,cosA_P2R*_Rod2len);//P1left P2right
            back_mid = Point_3(cosA_P1R*sinA_P2L*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2L*_Rod2len- cosA_P1R*_Rod1len,cosA_P2L*_Rod2len);//P1right P2left
            back_top = Point_3(cosA_P1R*sinA_P2R*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2R*_Rod2len- cosA_P1R*_Rod1len,cosA_P2R*_Rod2len);//P1right P2right
        }
    }
    else{
        back_down = Point_3(- sinA_P1L*_Rod1len,- cosA_P1L*_Rod1len,0);//P1left
        front_down = Point_3(- sinA_P1R*_Rod1len,- cosA_P1R*_Rod1len,0);//P1right
        if(P->_FD2Status == N90toP0){
            front_mid = Point_3(cosA_P1R*sinA_P2L*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2L*_Rod2len- cosA_P1R*_Rod1len,cosA_P2L*_Rod2len);//P1right P2left
            front_top = Point_3(cosA_P1R*sinA_P2R*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2R*_Rod2len- cosA_P1R*_Rod1len,cosA_P2R*_Rod2len);//P1right P2right
            back_mid = Point_3(cosA_P1L*sinA_P2L*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2L*_Rod2len- cosA_P1L*_Rod1len,cosA_P2L*_Rod2len);//P1left P2left
            back_top = Point_3(cosA_P1L*sinA_P2R*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2R*_Rod2len- cosA_P1L*_Rod1len,cosA_P2R*_Rod2len);//P1left P2right
        }
        else{
            assert(P->_FD2Status == N180toN90);
            front_mid = Point_3(cosA_P1R*sinA_P2R*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2R*_Rod2len- cosA_P1R*_Rod1len,cosA_P2R*_Rod2len);//P1right P2right
            front_top =  Point_3(cosA_P1R*sinA_P2L*_Rod2len- sinA_P1R*_Rod1len,-sinA_P1R*sinA_P2L*_Rod2len- cosA_P1R*_Rod1len,cosA_P2L*_Rod2len);//P1right P2left
            back_mid = Point_3(cosA_P1L*sinA_P2R*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2R*_Rod2len- cosA_P1L*_Rod1len,cosA_P2R*_Rod2len);//P1left P2right
            back_top =Point_3(cosA_P1L*sinA_P2L*_Rod2len- sinA_P1L*_Rod1len,-sinA_P1L*sinA_P2L*_Rod2len- cosA_P1L*_Rod1len,cosA_P2L*_Rod2len);//P1left P2left
        }
    }
    
    Q1=front_down;
    Q4=front_mid;
    Q7=Point_3(front_mid.x(),front_mid.y(),front_top.z());
    Q2=back_down;
    Q10=back_top;
    Q5=Q2+(Q4-Q1);
    Q8=Q2+(Q7-Q1);
    Q9=Q2+(front_top-Q1);
    
    Plane_3 back_plane(back_top,back_mid,back_down);
    Vector_3 dir(front_mid.y(),-front_mid.x(),0);
    
    Q3=back_plane.LineIntersection(Q1, dir);
    Q6=back_plane.LineIntersection(Q4, dir);
    Q11=back_plane.LineIntersection(Q7, dir);
    
    assert(Plane_3(Q1,Q2,Q4).WhichSide(Q5)==ON);
    assert(Plane_3(Q1,Q4,Q7).WhichSide(front_top)==ON);
    assert(Plane_3(Q2,Q5,Q8).WhichSide(Q9)==ON);
    assert(Plane_3(Q2,Q5,Q8).WhichSide(Q9)==ON);
    assert(Plane_3(Q6,Q4,Q7).WhichSide(Q11)==ON);
    assert(Plane_3(Q1,Q4,Q3).WhichSide(Q6)==ON);
    assert(Plane_3(Q6,Q4,Q7).WhichSide(Q11)==ON);
    assert(Plane_3(Q5,Q4,Q7).WhichSide(Q8)==ON);
        assert(back_plane.WhichSide(Q3)==ON and back_plane.WhichSide(Q6)==ON and back_plane.WhichSide(Q11)==ON);
    
    //选用p1mid p2mid的尾点做测试点
    _testPoint=Point_3(cosA(P->fd1_mid())*sinA(P->fd2_mid())*_Rod2len- sinA(P->fd1_mid())*_Rod1len,-sinA(P->fd1_mid())*sinA(P->fd2_mid())*_Rod2len- cosA(P->fd1_mid())*_Rod1len,cosA(P->fd2_mid())*_Rod2len);
    
    if(P->fd2_getL()==0 or P->fd2_getR()==0 or P->fd2_getR()==180 or P->fd2_getL()==-180){
        if(P->fd2_getL()==-90 or P->fd2_getL()==90 or P->fd2_getR()==-90 or P->fd2_getR()==90){
            build_S1S2S7coplanar_S4degenerate();
        }
        else{
            build_S4degenerate();
        }
    }
    else{
        build();
    }
    
//    for(auto it=_CEMRot->_PlaneSet.begin();it!=_CEMRot->_PlaneSet.end();it++){
//        assert((*it).WhichSide(front_mid)!=NEG and (*it).WhichSide(front_top)!=NEG and (*it).WhichSide(front_down)!=NEG);
//        assert((*it).WhichSide(back_down)!=NEG and (*it).WhichSide(back_top)!=NEG and (*it).WhichSide(back_mid)!=NEG);
//        assert((*it).WhichSide(Q1)!=NEG);
//        assert((*it).WhichSide(Q2)!=NEG);
//        assert((*it).WhichSide(Q3)!=NEG);
//        assert((*it).WhichSide(Q4)!=NEG);
//        assert((*it).WhichSide(Q5)!=NEG);
//        assert((*it).WhichSide(Q6)!=NEG);
//        assert((*it).WhichSide(Q7)!=NEG);
//        assert((*it).WhichSide(Q8)!=NEG);
//        assert((*it).WhichSide(Q9)!=NEG);
//        assert((*it).WhichSide(Q10)!=NEG);
//        assert((*it).WhichSide(Q11)!=NEG);
//    }
//    std::cout<<"Rod_2 done! "<<*P;
//    std::cout<<"Rod_2 :"<<*_CEMRot;
}
