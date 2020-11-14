//
//  Rod_2_build.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "Rod_2.hpp"
void Rod_2::build(){
    Vector_3 NVcandidate;
    double res;
    
    _CEMRot=std::make_shared<Polyhedra>(11,18,9);
    //S1
    NVcandidate=Cross(Q2-Q1, Q3-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S1]"<<std::endl;
    _CEMRot->addSegment(Q1, Q2);
    _CEMRot->addSegment(Q2, Q3);
    _CEMRot->addSegment(Q1, Q3);
    _CEMRot->addPoint(Q1);
    _CEMRot->addPoint(Q2);
    _CEMRot->addPoint(Q3);
    
    //S2
    NVcandidate=Cross(Q4-Q5, Q5-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S2]"<<std::endl;
    _CEMRot->addSegment(Q1, Q4);
    _CEMRot->addSegment(Q4, Q5);
    _CEMRot->addSegment(Q2, Q5);
    _CEMRot->addPoint(Q4);
    _CEMRot->addPoint(Q5);
    
    //S3
    NVcandidate=Cross(Q9-Q2, Q2-Q5);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S3]"<<std::endl;
    _CEMRot->addSegment(Q5, Q8);
    _CEMRot->addSegment(Q8, Q9);
    _CEMRot->addSegment(Q2, Q9);
    _CEMRot->addPoint(Q8);
    _CEMRot->addPoint(Q9);
    //S4
    NVcandidate=Cross(Q2-Q9, Q10-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S4]"<<std::endl;
    _CEMRot->addSegment(Q9, Q10);
    _CEMRot->addSegment(Q2, Q10);
    _CEMRot->addPoint(Q10);
    
    //S5
    NVcandidate=Cross(Q11-Q10, Q10-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S5]"<<std::endl;
    _CEMRot->addSegment(Q10, Q11);
    _CEMRot->addSegment(Q6, Q11);
    _CEMRot->addSegment(Q3, Q6);
    _CEMRot->addPoint(Q6);
    _CEMRot->addPoint(Q11);
    
    //S6
    NVcandidate=Cross(Q4-Q7, Q7-Q11);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S6]"<<std::endl;
    _CEMRot->addSegment(Q4, Q7);
    _CEMRot->addSegment(Q7, Q11);
    _CEMRot->addSegment(Q4, Q6);
    _CEMRot->addPoint(Q7);
    
    //S7
    NVcandidate=Cross(Q1-Q4, Q4-Q6);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S7]"<<std::endl;
    
    //S8
    NVcandidate=Cross(Q4-Q5, Q5-Q8);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S8]"<<std::endl;
    _CEMRot->addSegment(Q7, Q8);
    
    //S9
    NVcandidate=Cross(Q7-Q8, Q8-Q9);
    res=Dot(NVcandidate, _testPoint-Q7);
    if( res > 0 ) _CEMRot->addPlane( Q7 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q7 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S9]"<<std::endl;
}

void Rod_2::build_S1S2S7coplanar(){
    //验证
    Plane_3 base(Q1,Q2,Q3);
    assert(base.WhichSide(Q4)==ON);
    assert(base.WhichSide(Q5)==ON);
    assert(base.WhichSide(Q6)==ON);
    
    _CEMRot=std::make_shared<Polyhedra>(9,14,7);
    //构造
    Vector_3 NVcandidate;
    double res;
    //S1S2S7
    NVcandidate=Cross(Q2-Q5, Q5-Q4);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S1S2S7]"<<std::endl;
    _CEMRot->addSegment(Q2, Q5);
    _CEMRot->addSegment(Q4, Q5);
    _CEMRot->addSegment(Q4, Q6);
    _CEMRot->addSegment(Q2, Q6);
    _CEMRot->addPoint(Q2);
    _CEMRot->addPoint(Q4);
    _CEMRot->addPoint(Q5);
    _CEMRot->addPoint(Q6);
    
    //S3
    NVcandidate=Cross(Q9-Q2, Q2-Q5);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S3]"<<std::endl;
    _CEMRot->addSegment(Q5, Q8);
    _CEMRot->addSegment(Q8, Q9);
    _CEMRot->addSegment(Q2, Q9);
    _CEMRot->addPoint(Q8);
    _CEMRot->addPoint(Q9);
    
    //S4
    NVcandidate=Cross(Q2-Q9, Q10-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S4]"<<std::endl;
    _CEMRot->addSegment(Q9, Q10);
    _CEMRot->addSegment(Q2, Q10);
    _CEMRot->addPoint(Q10);
    
    //S5
    NVcandidate=Cross(Q11-Q10, Q10-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S5]"<<std::endl;
    _CEMRot->addSegment(Q10, Q11);
    _CEMRot->addSegment(Q6, Q11);
    _CEMRot->addPoint(Q11);
    
    //S6
    NVcandidate=Cross(Q4-Q7, Q7-Q11);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S6]"<<std::endl;
    _CEMRot->addSegment(Q4, Q7);
    _CEMRot->addSegment(Q7, Q11);
    _CEMRot->addPoint(Q7);
    
    //S8
    NVcandidate=Cross(Q4-Q5, Q5-Q8);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S8]"<<std::endl;
    _CEMRot->addSegment(Q7, Q8);
    
    //S9
    NVcandidate=Cross(Q7-Q8, Q8-Q9);
    res=Dot(NVcandidate, _testPoint-Q7);
    if( res > 0 ) _CEMRot->addPlane( Q7 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q7 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S9]"<<std::endl;
}

void Rod_2::build_S4degenerate(){
    //验证
    assert(Q9==Q10);
    
    _CEMRot=std::make_shared<Polyhedra>(10,16,8);
    //构造
    Vector_3 NVcandidate;
    double res;
    //S1
    NVcandidate=Cross(Q2-Q1, Q3-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S1]"<<std::endl;
    _CEMRot->addSegment(Q1, Q2);
    _CEMRot->addSegment(Q2, Q3);
    _CEMRot->addSegment(Q1, Q3);
    _CEMRot->addPoint(Q1);
    _CEMRot->addPoint(Q2);
    _CEMRot->addPoint(Q3);
    
    //S2
    NVcandidate=Cross(Q4-Q5, Q5-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S2]"<<std::endl;
    _CEMRot->addSegment(Q1, Q4);
    _CEMRot->addSegment(Q4, Q5);
    _CEMRot->addSegment(Q2, Q5);
    _CEMRot->addPoint(Q4);
    _CEMRot->addPoint(Q5);
    
    //S3
    NVcandidate=Cross(Q9-Q2, Q2-Q5);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S3]"<<std::endl;
    _CEMRot->addSegment(Q5, Q8);
    _CEMRot->addSegment(Q8, Q9);
    _CEMRot->addSegment(Q2, Q9);
    _CEMRot->addPoint(Q8);
    _CEMRot->addPoint(Q9);

    //S5
    NVcandidate=Cross(Q11-Q9, Q9-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S5]"<<std::endl;
    _CEMRot->addSegment(Q9, Q11);
    _CEMRot->addSegment(Q6, Q11);
    _CEMRot->addSegment(Q3, Q6);
    _CEMRot->addPoint(Q6);
    _CEMRot->addPoint(Q11);
    
    //S6
    NVcandidate=Cross(Q4-Q7, Q7-Q11);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S6]"<<std::endl;
    _CEMRot->addSegment(Q4, Q7);
    _CEMRot->addSegment(Q7, Q11);
    _CEMRot->addSegment(Q4, Q6);
    _CEMRot->addPoint(Q7);
    
    //S7
    NVcandidate=Cross(Q1-Q4, Q4-Q6);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S7]"<<std::endl;
    
    //S8
    NVcandidate=Cross(Q4-Q5, Q5-Q8);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S8]"<<std::endl;
    _CEMRot->addSegment(Q7, Q8);
    
    //S9
    NVcandidate=Cross(Q7-Q8, Q8-Q9);
    res=Dot(NVcandidate, _testPoint-Q7);
    if( res > 0 ) _CEMRot->addPlane( Q7 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q7 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S9]"<<std::endl;
}

void Rod_2::build_S1S2S7coplanar_S4degenerate(){
    //验证
    Plane_3 base(Q1,Q2,Q3);
    assert(base.WhichSide(Q4)==ON);
    assert(base.WhichSide(Q5)==ON);
    assert(base.WhichSide(Q6)==ON);
    //验证
    assert(Q9==Q10);
    
    _CEMRot=std::make_shared<Polyhedra>(8,12,6);
    //构造
    Vector_3 NVcandidate;
    double res;
    //S1S2S7
    NVcandidate=Cross(Q2-Q5, Q5-Q4);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S1S2S7]"<<std::endl;
    _CEMRot->addSegment(Q2, Q5);
    _CEMRot->addSegment(Q4, Q5);
    _CEMRot->addSegment(Q4, Q6);
    _CEMRot->addSegment(Q2, Q6);
    _CEMRot->addPoint(Q2);
    _CEMRot->addPoint(Q4);
    _CEMRot->addPoint(Q5);
    _CEMRot->addPoint(Q6);
    
    //S3
    NVcandidate=Cross(Q9-Q2, Q2-Q5);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S3]"<<std::endl;
    _CEMRot->addSegment(Q5, Q8);
    _CEMRot->addSegment(Q8, Q9);
    _CEMRot->addSegment(Q2, Q9);
    _CEMRot->addPoint(Q8);
    _CEMRot->addPoint(Q9);
    
    //S5
    NVcandidate=Cross(Q11-Q9, Q9-Q2);
    res=Dot(NVcandidate, _testPoint-Q2);
    if( res > 0 ) _CEMRot->addPlane( Q2 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q2 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S5]"<<std::endl;
    _CEMRot->addSegment(Q9, Q11);
    _CEMRot->addSegment(Q6, Q11);
    _CEMRot->addPoint(Q11);
    
    //S6
    NVcandidate=Cross(Q4-Q7, Q7-Q11);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S6]"<<std::endl;
    _CEMRot->addSegment(Q4, Q7);
    _CEMRot->addSegment(Q7, Q11);
    _CEMRot->addPoint(Q7);
    
    //S8
    NVcandidate=Cross(Q4-Q5, Q5-Q8);
    res=Dot(NVcandidate, _testPoint-Q4);
    if( res > 0 ) _CEMRot->addPlane( Q4 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q4 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S8]"<<std::endl;
    _CEMRot->addSegment(Q7, Q8);
    
    //S9
    NVcandidate=Cross(Q7-Q8, Q8-Q9);
    res=Dot(NVcandidate, _testPoint-Q7);
    if( res > 0 ) _CEMRot->addPlane( Q7 , NVcandidate);
    else if(res < 0) _CEMRot->addPlane( Q7 , -NVcandidate);
    else std::cout<<"Wrong _testPoint [S9]"<<std::endl;
}
