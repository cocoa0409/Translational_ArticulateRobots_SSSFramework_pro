//
//  Rod_1.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Rod_1_hpp
#define Rod_1_hpp
#include "Rod_base.hpp"
class Configbox;

class Rod_1 : public Rod_base{
public:
    Point_3 F1,F2,F3,F4;
    Rod_1(){}
    Rod_1(ParaBox * P);
    ~Rod_1(){}
    void build();
};

#endif /* Rod_1_hpp */
