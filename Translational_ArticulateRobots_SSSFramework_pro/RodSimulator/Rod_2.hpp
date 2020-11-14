//
//  Rod_2.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Rod_2_hpp
#define Rod_2_hpp
#include "Rod_base.hpp"

class Configbox;

class Rod_2 : public Rod_base{
public:
    Point_3 Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11;
    Point_3 _testPoint;
    Rod_2(){}
    Rod_2(ParaBox * P);
    ~Rod_2(){}
    
    void build();
    void build_S1S2S7coplanar();
    void build_S4degenerate();
    void build_S1S2S7coplanar_S4degenerate();
};


#endif /* Rod_2_hpp */
