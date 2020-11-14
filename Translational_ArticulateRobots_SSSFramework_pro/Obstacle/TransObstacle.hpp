//
//  TransObstacle.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/6.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef TransObstacle_hpp
#define TransObstacle_hpp
#include "../CONFIG.h"
#include "../Box/ParaBox.hpp"
#include "../Geo/Triangle_3.hpp"
#include "../Geo/Polyhedra.hpp"

class TransObstacle{
public:
    static std::vector<std::shared_ptr<Polyhedra>> _baseObstacle;
    static void Build_base_TriangleObstacle(const Point_3 & p1,const Point_3 & p2,const Point_3 & p3);
    
    TransObstacle(){}
    TransObstacle(ParaBox * P);
    ~TransObstacle(){
        for(auto it=_TransObstacle.begin();it!=_TransObstacle.end();it++)
            (*it).reset();
    }
    
    void ConstructMinkowskiSumObstacle(const std::shared_ptr<Polyhedra> p,IntervalT<double> & Ix,IntervalT<double> & Iy);
    
    std::vector<std::shared_ptr<Polyhedra>> _TransObstacle;
    std::vector<std::vector<std::shared_ptr<Triangle_3>>> _StuckObstacle;
};

#endif /* TransObstacle_hpp */
