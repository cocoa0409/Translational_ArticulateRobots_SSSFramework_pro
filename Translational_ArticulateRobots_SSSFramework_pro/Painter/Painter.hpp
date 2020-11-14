//
//  Painter.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/6/10.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Painter_hpp
#define Painter_hpp

#include "../CONFIG.h"
#include "../Obstacle/TransObstacle.hpp"
#include "../SSS/SSS.hpp"
class Painter{
public:
    std::ofstream outfile;
    Painter(){
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
//        outfile.open("/Users/choukichiou/Desktop/Painter.SSSout",std::ios::out);
    }
    ~Painter(){
//        outfile.close();
    }
    
    void Triangle_Painter(std::shared_ptr<Polyhedra> t);
    void _baseObstacle_Painter();
    
    void Bound_Painter(SSS & sss);
    
    void Path_Painter(SSS & sss);
    
};

#endif /* Painter_hpp */
