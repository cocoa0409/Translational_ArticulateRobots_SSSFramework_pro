//
//  Painter.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/6/10.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Painter.hpp"
void Painter::Triangle_Painter(std::shared_ptr<Polyhedra> t){
    Point_3 p1=t->_PointSet[0];
    Point_3 p2=t->_PointSet[1];
    Point_3 p3=t->_PointSet[2];
    std::cout<<"ParametricPlot3D((("<<p1.x()<<")*s + ("<<p2.x()<<")*(1-s))*t + ("<<p3.x()<<")*(1-t),(("<<p1.y()<<")*s + ("<<p2.y()<<")*(1-s))*t + ("<<p3.y()<<")*(1-t),(("<<p1.z()<<")*s + ("<<p2.z()<<")*(1-s))*t + ("<<p3.z()<<")*(1-t),t=[0,1],s=[0,1],color=yellow,solid=0)"<<std::endl;
}

void Painter::_baseObstacle_Painter(){
    std::cout<<std::endl<<"// [Obstacle Printer] "<<std::endl;
    for(auto it=TransObstacle::_baseObstacle.begin();it!=TransObstacle::_baseObstacle.end();it++){
        Triangle_Painter(*it);
    }
}


void Painter::Bound_Painter(SSS & sss){
    IntervalT<double> IxBound = sss._IxBound;
    IntervalT<double> IyBound = sss._IyBound;
    
    std::cout<<std::endl<<"// [Bound Printer] "<<std::endl;
    std::cout<<"ParametricPlot3D(("<<IxBound.getL()<<")*s + ("<<IxBound.getR()<<")*(1-s),"<<IyBound.getL()<<",0,s=[0,1],color=red,lines=3)"<<std::endl;
    std::cout<<"ParametricPlot3D(("<<IxBound.getL()<<")*s + ("<<IxBound.getR()<<")*(1-s),"<<IyBound.getR()<<",0,s=[0,1],color=red,lines=3)"<<std::endl;
    std::cout<<"ParametricPlot3D("<<IxBound.getL()<<",("<<IyBound.getL()<<")*s + ("<<IyBound.getR()<<")*(1-s)"<<",0,s=[0,1],color=red,lines=3)"<<std::endl;
    std::cout<<"ParametricPlot3D("<<IxBound.getR()<<",("<<IyBound.getL()<<")*s + ("<<IyBound.getR()<<")*(1-s)"<<",0,s=[0,1],color=red,lines=3)"<<std::endl;
}


void Painter::Path_Painter(SSS & sss){
    std::cout<<std::endl<<"// [Path Printer] "<<std::endl;
    std::cout<<"len1="<<_Rod1len<<std::endl;
    std::cout<<"len2="<<_Rod2len<<std::endl<<std::endl;
    std::string x,y,fd1,fd2;
    x="x=[";
    y="y=[";
    fd1="fd1=[";
    fd2="fd2=[";
    
    double prex= sss._Psrc[0];
    double prey= sss._Psrc[1];
    
    for(auto it=SSS::_RoadPoints.begin();it!=SSS::_RoadPoints.end();it++){
        if(it!=SSS::_RoadPoints.begin()){
            x.append(",");
            y.append(",");
            fd1.append(",");
            fd2.append(",");
        }
        x.append(std::to_string((*it)[0]));
        y.append(std::to_string((*it)[1]));
        fd1.append(std::to_string((*it)[2]));
        fd2.append(std::to_string((*it)[3]));
        
        if((*it)[0]!=prex or (*it)[1]!=prey){
            std::cout<<"ParametricPlot3D("<<prex<<"+t*("<<(*it)[0]<<"-("<<prex<<")),"<<prey<<"+t*("<<(*it)[1]<<"-("<<prey<<")),0,t=[0,1],color=black,lines=3)"<<std::endl;
            prex=(*it)[0];
            prey=(*it)[1];
        }
        
        
    }
    x.append("]");
    y.append("]");
    fd1.append("]/180*@pi");
    fd2.append("]/180*@pi");
    
    std::cout<<std::endl<<x<<std::endl<<y<<std::endl<<fd1<<std::endl<<fd2<<std::endl<<std::endl;
    
    std::cout<<std::endl<<"ParametricPlot3D(x(1)+t*(-sin(fd1(1))),y(1)+t*(-cos(fd1(1))),0,t=[0,len1],color=green,lines=10)"<<std::endl;
    std::cout<<"ParametricPlot3D(x("<<SSS::_RoadPoints.size()<<")+t*(-sin(fd1("<<SSS::_RoadPoints.size()<<"))),y("<<SSS::_RoadPoints.size()<<")+t*(-cos(fd1("<<SSS::_RoadPoints.size()<<"))),0,t=[0,len1],color=black,lines=10)"<<std::endl;
    
    std::cout<<"v2=(cos(fd2(1))*[0,0,1]+sin(fd2(1))*Cross([0,0,1],[-sin(fd1(1)),-cos(fd1(1)),0]))"<<std::endl;
    std::cout<<"ParametricPlot3D(x(1)-sin(fd1(1))*len1 + t*(v2(1)), y(1)-cos(fd1(1))*len1 + t*(v2(2)),t*v2(3),t=[0,len2],color=blue,lines=10)"<<std::endl;
    std::cout<<"v2=(cos(fd2("<<SSS::_RoadPoints.size()<<"))*[0,0,1]+sin(fd2("<<SSS::_RoadPoints.size()<<"))*Cross([0,0,1],[-sin(fd1("<<SSS::_RoadPoints.size()<<")),-cos(fd1("<<SSS::_RoadPoints.size()<<")),0]))"<<std::endl;
    std::cout<<"ParametricPlot3D(x("<<SSS::_RoadPoints.size()<<")-sin(fd1("<<SSS::_RoadPoints.size()<<"))*len1 + t*(v2(1)), y("<<SSS::_RoadPoints.size()<<")-cos(fd1("<<SSS::_RoadPoints.size()<<"))*len1 + t*(v2(2)),t*v2(3),t=[0,len2],color=black,lines=10)"<<std::endl<<std::endl;
    
    
    std::cout<<"Animate(n, 1.."<<SSS::_RoadPoints.size()<<"-> 0.5)"<<std::endl;
    std::cout<<"idx=floor(n)"<<std::endl;
    std::cout<<"time=n-floor(n)"<<std::endl;
    std::cout<<"TimeStepx   = (1-time)* x(idx)+ time * x(idx+1)"<<std::endl;
    std::cout<<"TimeStepy   = (1-time)* y(idx)+ time * y(idx+1)"<<std::endl;
    std::cout<<"TimeStepfd1 = (1-time)* fd1(idx)+ time *fd1(idx+1)"<<std::endl;
    std::cout<<"TimeStepfd2 = (1-time)* fd2(idx)+ time *fd2(idx+1)"<<std::endl;
    std::cout<<"end0=[TimeStepx,TimeStepy,0]"<<std::endl;
    std::cout<<"end1= end0 + [-sin(TimeStepfd1),-cos(TimeStepfd1),0] * len1"<<std::endl;
    std::cout<<"end2= end1 + (cos(TimeStepfd2)*[0,0,1]+sin(TimeStepfd2)*Cross([0,0,1],[-sin(TimeStepfd1),-cos(TimeStepfd1),0]))*len2"<<std::endl;
    std::cout<<"ParametricPlot3D(end0(1)+t*(end1(1)-end0(1)),end0(2)+t*(end1(2)-end0(2)),end0(3)+t*(end1(3)-end0(3)),t=[0,1],color=green,lines=10)"<<std::endl;
    std::cout<<"ParametricPlot3D(end1(1)+t*(end2(1)-end1(1)),end1(2)+t*(end2(2)-end1(2)),end1(3)+t*(end2(3)-end1(3)),t=[0,1],color=blue,lines=10)"<<std::endl;
}
