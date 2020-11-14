//
//  Rod_base.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Rod_base_hpp
#define Rod_base_hpp

#include "../CONFIG.h"
#include "../Geo/Vector_3.hpp"
#include "../Geo/Segment_3.hpp"
#include "../Geo/Plane_3.hpp"
#include "../Geo/Polyhedra.hpp"
#include "../Box/ParaBox.hpp"

class Rod_base{
public:
    std::shared_ptr<Polyhedra> _CEMRot;
    ~Rod_base(){_CEMRot.reset();}
};
double sinA(double x);
double cosA(double x);

#endif /* Rod_base_hpp */
