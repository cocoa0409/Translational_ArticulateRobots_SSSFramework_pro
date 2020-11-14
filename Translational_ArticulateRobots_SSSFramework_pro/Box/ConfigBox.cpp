//
//  ConfigBox.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "ConfigBox.hpp"

PathPlanningMode ConfigBox::_PathPlanningMode=Insitu;

std::unordered_map<ParaBox, std::shared_ptr<TransObstacle>,TransHashFunc,TransEqual> ConfigBox::TransRepo;
std::unordered_map<ParaBox, std::shared_ptr<Rod_1>,Rot_1HashFunc,Rod_1Equal> ConfigBox::Rod_1Repo;
std::unordered_map<ParaBox, std::shared_ptr<Rod_2>,Rot_2HashFunc,Rod_2Equal> ConfigBox::Rod_2Repo;
ConfigBox::ConfigBox(ParaBox * P){
    //如果已经通过ParaBox * P初始化了一个ConfigBox,那么不可以再初始化
    if(P->_ConfigBox != nullptr){
        return;
    }
    _P=P;
    P->_ConfigBox=this;
    if(P->_FD2Status == INI){
        std::cout<<"No Way to initialize a ConfigBox from _FD2Status=INI"<<std::endl;
        return;
    }
    //从Repo中提取信息,或第一次初始化
    
    if(_PathPlanningMode == Movable){
        if(TransRepo.count(*P)!=0){//一样的x y参数 对应一样的TransObstacle
            _TransObstacle = TransRepo[*P];
        }
        else{
            _TransObstacle=std::make_shared<TransObstacle>(P);
            TransRepo[*P]= _TransObstacle;
        }
    }
    
    if(Rod_1Repo.count(*P)!=0){
        _Rod_1 = Rod_1Repo[*P];
    }
    else{
        _Rod_1=std::make_shared<Rod_1>(P);
        Rod_1Repo[*P]= _Rod_1;
    }
    
    if(Rod_2Repo.count(*P)!=0){
        _Rod_2 = Rod_2Repo[*P];
    }
    else{
        _Rod_2=std::make_shared<Rod_2>(P);
        Rod_2Repo[*P]= _Rod_2;
    }
}
ConfigBoxStatus ConfigBox::isCollide_Rod_1(){
    auto trueit=std::find(_P->_ObssJudgeFlag_Rod_1.begin(),_P->_ObssJudgeFlag_Rod_1.end(),true);
    if(trueit==_P->_ObssJudgeFlag_Rod_1.end()){
        return BFREE;
    }
    else{
        for(auto it=_P->_ObssJudgeFlag_Rod_1.begin();it!=_P->_ObssJudgeFlag_Rod_1.end();it++){
            if(*it==true){//当需要检测时
                if(_PathPlanningMode == Insitu){
                    auto Obssit=TransObstacle::_baseObstacle.begin()+( it-_P->_ObssJudgeFlag_Rod_1.begin() );
                    if(isIntersected(*(_Rod_1->_CEMRot), **Obssit)){
                        return BMIXED;
                    }
                    else{
                        *it=false;
                    }
                }
                else{//(_PathPlanningMode == Movable)
                    auto Obssit=_TransObstacle->_TransObstacle.begin()+( it-_P->_ObssJudgeFlag_Rod_1.begin() );
                    if(isIntersected(*(_Rod_1->_CEMRot), **Obssit)){
//                        //增加STUCK测试
                        if(IncludeBSTUCKtest){
                            auto StuckVit=_TransObstacle->_StuckObstacle.begin()+( it-_P->_ObssJudgeFlag_Rod_1.begin() );
                            for(auto Stuckit=(*StuckVit).begin();Stuckit!=(*StuckVit).end();Stuckit++){
                                if(isIntersected(**Stuckit, Segment_3(_Rod_1->F1,_Rod_1->F2))==false or isIntersected(**Stuckit, Segment_3(_Rod_1->F1,_Rod_1->F4))==false){
                                    return BMIXED;
                                }
                            }
                            return BSTUCK;
                        }
                        else{
                            return BMIXED;
                        }
                    }
                    else{
                        *it=false;
                    }
                }
            }
        }
    }
    return BFREE;
}
    
ConfigBoxStatus ConfigBox::isCollide_Rod_2(){
    auto trueit=std::find(_P->_ObssJudgeFlag_Rod_2.begin(),_P->_ObssJudgeFlag_Rod_2.end(),true);
    if(trueit==_P->_ObssJudgeFlag_Rod_2.end()){
        return BFREE;
    }
    else{
        for(auto it=_P->_ObssJudgeFlag_Rod_2.begin();it!=_P->_ObssJudgeFlag_Rod_2.end();it++){
            if(*it==true){//当需要检测时
                if(_PathPlanningMode == Insitu){
                    auto Obssit=TransObstacle::_baseObstacle.begin()+( it-_P->_ObssJudgeFlag_Rod_2.begin() );
                    if(isIntersected(*(_Rod_2->_CEMRot), **Obssit)){
                        return BMIXED;
                    }
                    else{
                        *it=false;
                    }
                }
                else{//(_PathPlanningMode == Movable)
                    auto Obssit=_TransObstacle->_TransObstacle.begin()+( it-_P->_ObssJudgeFlag_Rod_2.begin() );
                    if(isIntersected(*(_Rod_2->_CEMRot), **Obssit)){
                        //                        //增加STUCK测试
                        if(IncludeBSTUCKtest){
                            auto StuckVit=_TransObstacle->_StuckObstacle.begin()+( it-_P->_ObssJudgeFlag_Rod_2.begin() );
                            for(auto Stuckit=(*StuckVit).begin();Stuckit!=(*StuckVit).end();Stuckit++){
                                if(isIntersected(**Stuckit, Segment_3(_Rod_2->Q1,_Rod_2->Q4))==false or isIntersected(**Stuckit, Segment_3(_Rod_2->Q2,_Rod_2->Q5))==false or isIntersected(**Stuckit, Segment_3(_Rod_2->Q2,_Rod_2->Q9))==false or isIntersected(**Stuckit, Segment_3(_Rod_2->Q2,_Rod_2->Q10))==false or isIntersected(**Stuckit, Segment_3(_Rod_2->Q3,_Rod_2->Q6))==false){
                                    return BMIXED;
                                }
                            }
                            return BSTUCK;
                        }
                        else{
                            return BMIXED;
                        }
                    }
                    else{
                        *it=false;
                    }
                }
            }
        }
    }
    return BFREE;
}

ConfigBoxStatus ConfigBox::predicate(){
    if(isCollide_Rod_1()==BMIXED or isCollide_Rod_2()==BMIXED){
        _P->_ConfigBoxStatus=BMIXED;
    }
    else if(isCollide_Rod_1()==BSTUCK or isCollide_Rod_2()==BSTUCK){
        _P->_ConfigBoxStatus=BSTUCK;
    }
    else{
        _P->_ConfigBoxStatus=BFREE;
    }
    return _P->_ConfigBoxStatus;
}
