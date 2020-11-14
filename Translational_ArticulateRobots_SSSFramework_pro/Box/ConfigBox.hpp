//
//  ConfigBox.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef ConfigBox_hpp
#define ConfigBox_hpp

#include "../CONFIG.h"
#include "ParaBox.hpp"
#include "hashmapUtil.h"
#include "../RodSimulator/Rod_1.hpp"
#include "../RodSimulator/Rod_2.hpp"
#include "../Obstacle/TransObstacle.hpp"

class ConfigBox{
public:
    static PathPlanningMode _PathPlanningMode;
    static std::unordered_map<ParaBox, std::shared_ptr<TransObstacle>,TransHashFunc,TransEqual> TransRepo;
    static std::unordered_map<ParaBox, std::shared_ptr<Rod_1>,Rot_1HashFunc,Rod_1Equal> Rod_1Repo;
    static std::unordered_map<ParaBox, std::shared_ptr<Rod_2>,Rot_2HashFunc,Rod_2Equal> Rod_2Repo;
    ParaBox * _P;
    std::shared_ptr<Rod_1> _Rod_1;
    std::shared_ptr<Rod_2> _Rod_2;
    std::shared_ptr<TransObstacle> _TransObstacle;
    
    ConfigBox(ParaBox * P);
    ~ConfigBox(){
        _P->_ConfigBox=nullptr;
        _Rod_1.reset();
        _Rod_2.reset();
        _TransObstacle.reset();
//        std::cout<<"* Destroy ConfigBox of ParaBox:"<< _P->_id <<std::endl;
//        std::cout<<" Rod_1["<<_Rod_1<<"].use_count (before reset) : "<<_Rod_1.use_count()<<std::endl;
//        std::cout<<" Rod_2["<<_Rod_2<<"].use_count (before reset) : "<<_Rod_2.use_count()<<std::endl;
//        std::cout<<" Trans["<<_TransObstacle<<"].use_count (before reset) : "<<_TransObstacle.use_count()<<std::endl;
    }
    ConfigBoxStatus isCollide_Rod_1();
    ConfigBoxStatus isCollide_Rod_2();
    ConfigBoxStatus predicate();
    
};

#endif /* ConfigBox_hpp */
