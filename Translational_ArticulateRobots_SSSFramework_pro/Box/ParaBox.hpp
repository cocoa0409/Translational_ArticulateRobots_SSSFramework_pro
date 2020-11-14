//
//  ParaBox.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef ParaBox_hpp
#define ParaBox_hpp
#include "../CONFIG.h"
#include "IntervalT.h"

class TransObstacle;
class ConfigBox;
class dSet;

class ParaBox{
public:
    static size_t _id_count;
    static size_t _obs_size;
    
    size_t _id =0;
    int _depth = 0;
    IntervalT<double> _Ix;
    IntervalT<double> _Iy;
    IntervalT<double> _I1;
    IntervalT<double> _I2;
    FD2Status _FD2Status;
    
    ParaBox * _parent=nullptr;
    std::vector<ParaBox *> _childrens;
    std::vector<ParaBox *> _neighbors;
    
    std::vector<bool> _ObssJudgeFlag_Rod_1;
    std::vector<bool> _ObssJudgeFlag_Rod_2;
    
    ConfigBox * _ConfigBox=nullptr;
    ConfigBoxStatus _ConfigBoxStatus=BUNKNOWN;
    
    dSet * pSet=nullptr;
    
    ParaBox(){}
    ParaBox(ParaBox * parent,const IntervalT<double> & Ix,const IntervalT<double> & Iy,const IntervalT<double> & I1,const IntervalT<double> & I2);
    ~ParaBox(){}
    
    bool isNeighbor(ParaBox * P);
    bool isLeaf();
    bool isIn(std::vector<double> & v);
    bool SPLIT(double eps_trans,double eps_rot);
    
    double fdx_getL() const {return _Ix.getL();}
    double fdx_getR() const {return _Ix.getR();}
    double fdx_mid() const {return _Ix.mid();}
    double fdx_width() const {return _Ix.width();}
    
    double fdy_getL() const {return _Iy.getL();}
    double fdy_getR() const {return _Iy.getR();}
    double fdy_mid() const {return _Iy.mid();}
    double fdy_width() const {return _Iy.width();}
    
    double fd1_getL() const {return _I1.getL();}
    double fd1_getR() const {return _I1.getR();}
    double fd1_mid() const {return _I1.mid();}
    double fd1_width() const {return _I1.width();}
    
    double fd2_getL() const {return _I2.getL();}
    double fd2_getR() const {return _I2.getR();}
    double fd2_mid() const {return _I2.mid();}
    double fd2_width() const {return _I2.width();}
    friend std::ostream & operator<<(std::ostream &out, ParaBox & P);
};

std::vector<double> FindRoadPoint(ParaBox * a,ParaBox * b);

#endif /* ParaBox_hpp */
