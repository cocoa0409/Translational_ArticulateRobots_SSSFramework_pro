//
//  SSS.hpp
//  Translational_ArticulateRobots_SSSFramework
//
//  Created by 兆吉 王 on 2020/4/29.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef SSS_hpp
#define SSS_hpp
#include "../CONFIG.h"
#include "../Box/ParaBox.hpp"
#include "../Box/ConfigBox.hpp"
#include "../UnionFind/UnionFind.hpp"

struct cmpDepthSmaller{
    bool operator ()(ParaBox * a,ParaBox * b){
        if(a->_depth != b->_depth)
            return a->_depth > b->_depth;//depth小值优先
        else
            return a->_id > b->_id;//id小优先
    }
};

class SSS{
public:
    std::vector<double> _Psrc;
    std::vector<double> _Pdst;
    double _epsTrans;
    double _epsRot;
    IntervalT<double> _IxBound;
    IntervalT<double> _IyBound;
    
    std::vector<std::pair<bool,bool>> _PRE_SPLIT;
    std::vector<int> _Goal_Depth;
    
    std::priority_queue<ParaBox *,std::vector<ParaBox *>,cmpDepthSmaller> _Q;
    
    static std::vector<ParaBox *> _RoadBoxes;
    static std::vector<std::vector<double>> _RoadPoints;
    
    
    ParaBox * ParaBoxPsrc=nullptr;
    ParaBox * ParaBoxPdst=nullptr;
    
    SSS(){}
    ~SSS(){}
    void Setup_RodLength(double rl1,double rl2);
    void Setup_Bound(const IntervalT<double> & IxBound,const IntervalT<double> & IyBound,double epsTrans,double epsRot);
    void Setup_PathPlanningMode(PathPlanningMode ppm);
    void Setup_IncludeBSTUCKtest(bool iBSTUCKt);
    void Setup_Goal(const std::vector<double> & Psrc,const std::vector<double> & Pdst);
    
    void Setup_PRE_Subdivision_rule(const std::pair<bool, bool> & presplit,int goaldepth);
    
    void Build_base_TriangleObstacle(const Point_3 & a,const Point_3 & b,const Point_3 & c);
    
    void printSetupLog();
    
    void clearRod_1Repo(){ConfigBox::Rod_1Repo.clear();}
    void clearRod_2Repo(){ConfigBox::Rod_2Repo.clear();}
    void clearTransRepo(){ConfigBox::TransRepo.clear();}
    void PRESubdivision(bool splitTrans,bool splitRot,int layer);
    bool FindPath();
    bool BuildRoadMap();
};

#endif /* SSS_hpp */
