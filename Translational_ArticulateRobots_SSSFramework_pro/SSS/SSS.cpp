//
//  SSS.cpp
//  Translational_ArticulateRobots_SSSFramework
//
//  Created by 兆吉 王 on 2020/4/29.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "SSS.hpp"

void SSS::Setup_PathPlanningMode(PathPlanningMode ppm){
    ConfigBox::_PathPlanningMode = ppm;
    if(ConfigBox::_PathPlanningMode == Insitu){
        _epsTrans=INT_MAX;//禁止平移细分
    }
}

void SSS::Setup_IncludeBSTUCKtest(bool iBSTUCKt){
    IncludeBSTUCKtest = iBSTUCKt;
}

void SSS::Setup_RodLength(double rl1,double rl2){
    _Rod1len = rl1;
    _Rod2len = rl2;
}
void SSS::Setup_Bound(const IntervalT<double> & IxBound,const IntervalT<double> & IyBound,double epsTrans,double epsRot){
    _IxBound=IxBound;
    _IyBound=IyBound;
    _epsTrans=epsTrans;
    _epsRot=epsRot;
    
}
void SSS::Setup_Goal(const std::vector<double> & Psrc,const std::vector<double> & Pdst){
    _Psrc.clear();
    std::copy(Psrc.begin(),Psrc.end(),std::back_inserter(_Psrc));
    _Pdst.clear();
    std::copy(Pdst.begin(),Pdst.end(),std::back_inserter(_Pdst));
}

void SSS::Build_base_TriangleObstacle(const Point_3 & a,const Point_3 & b,const Point_3 & c){
    TransObstacle::Build_base_TriangleObstacle(a, b, c);
}

void SSS::Setup_PRE_Subdivision_rule(const std::pair<bool, bool> & presplit,int goaldepth){
    _PRE_SPLIT.push_back(presplit);
    _Goal_Depth.push_back(goaldepth);
}



void SSS::printSetupLog(){
    std::cout<<std::endl<<"|---------- <<SSS Log>> ----------"<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"| PathPlanning Mode: ["<<(ConfigBox::_PathPlanningMode==Insitu?"Insitu]":"Movable]")<<std::endl;
    std::cout<<"| Enable BSTUCK test ? : ["<<(IncludeBSTUCKtest==true?"True]":"False]")<<std::endl;
    std::cout<<"| Rod1len = "<<_Rod1len<<" ; Rod2len = "<<_Rod2len<<std::endl;
    std::cout<<"| IxBound = "<<_IxBound<<" ; IyBound = "<<_IyBound<<std::endl;
    std::cout<<"| epsTrans = "<<_epsTrans<<" ; expRot = "<<_epsRot<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"| FROM ("<<_Psrc[0]<<"_x, \t"<<_Psrc[1]<<"_y, \t"<<_Psrc[2]<<"_rod1, \t"<<_Psrc[3]<<"_rod2)"<<std::endl;
    std::cout<<"| TO   ("<<_Pdst[0]<<"_x, \t"<<_Pdst[1]<<"_y, \t"<<_Pdst[2]<<"_rod1, \t"<<_Pdst[3]<<"_rod2)"<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"| ["<<(TransObstacle::_baseObstacle).size()<<"] Triangle in Environment"<<std::endl;
    std::cout<<"| ["<<ParaBox::_obs_size<<"] in ParaBox Memory"<<std::endl;
    for(auto it=TransObstacle::_baseObstacle.begin();it!=TransObstacle::_baseObstacle.end();it++){
        std::cout<<"| ";
        for(auto itp=(*it)->_PointSet.begin();itp!=(*it)->_PointSet.end();itp++){
            std::cout<<*itp;
            if(itp!=(*it)->_PointSet.end()-1) std::cout<<" <-> ";
        }
        std::cout<<std::endl;
    }
    std::cout<<"|"<<std::endl<<"| sizeof(ParaBox) = " <<sizeof(ParaBox)<<" ; sizeof(ConfigBox) = "<<sizeof(ConfigBox)<<std::endl;
    std::cout<<"|----------"<<std::endl<<std::endl;
}

void SSS::PRESubdivision(bool SPLITTrans,bool SPLITRot,int GoalDepth){
    assert(SPLITTrans or SPLITRot);
    if(SPLITTrans) assert(ConfigBox::_PathPlanningMode==Movable);
    
    ParaBox * cParaBox;
//    std::priority_queue<ParaBox *,std::vector<ParaBox *>,cmpDepthSmaller> _Q_clearner;
/*
 PRESubdivision直到_Q中所有的盒子的深度均为GoalDepth
 */
    int preDepth=_Q.top()->_depth-1;
    
    if(SPLITTrans and SPLITRot) std::cout<<"| *Subdivide (Ix Iy I1 I2) FROM depth ["<<preDepth+1<<"] TO depth ["<<GoalDepth<<"]";
    else if(SPLITTrans) std::cout<<"| *Subdivide (Ix Iy) FROM depth ["<<preDepth+1<<"] TO depth ["<<GoalDepth<<"]";
        else std::cout<<"| *Subdivide (I1 I2) FROM depth ["<<preDepth+1<<"] TO depth ["<<GoalDepth<<"]";
    std::cout<<std::endl;
    
    while( _Q.top()->_depth<GoalDepth){
        if(_Q.top()->_depth!=preDepth){
            std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
            preDepth=_Q.top()->_depth;
        }
        cParaBox = _Q.top();
        _Q.pop();
        assert(cParaBox->SPLIT(SPLITTrans?_epsTrans:DBL_MAX, SPLITRot?_epsRot:DBL_MAX));
        for(auto it=cParaBox->_childrens.begin();it!=cParaBox->_childrens.end();it++){
            _Q.push(*it);
        }
//        _Q_clearner.push(cParaBox);
///*
// clearner完成的工作是,当前优先队列最小深度的盒子为N,那么清除所有深度小于N-2的盒子
// 这是由于均匀细分策略的选择,深度为N盒子的所有 邻居 , 深度至少为 N-1
// */
    }
    std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
//    while(_Q_clearner.top()->_depth < _Q.top()->_depth-1 ){
//        cParaBox = _Q_clearner.top();
//        _Q_clearner.pop();
//        delete cParaBox;
//    }
    
}



bool SSS::FindPath(){
    
    clock_t time_start=clock();
    std::cout<<"|----------"<<std::endl<<"|"<<std::endl;
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] Finding a path ..."<<std::endl;
    std::cout<<"|"<<std::endl;
    
    
    std::cout<<"|-1--------"<<std::endl<<"|"<<std::endl;
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] PRE-Subdividing ..."<<std::endl;
    _Q.push(new ParaBox(nullptr,_IxBound,_IyBound,{-180,180},{-180,180}));
    for(auto it=_PRE_SPLIT.begin();it!=_PRE_SPLIT.end();it++){
        PRESubdivision(it->first, it->second, *(_Goal_Depth.begin()+(it-_PRE_SPLIT.begin())));
    }
    
    
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] PRE-Subdivision OVER "<<std::endl;
    std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"|-2--------"<<std::endl<<"|"<<std::endl;
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] Finding BFREE box containing Psrc & Pdst ..."<<std::endl;
    ParaBox * cParaBox;
//    ParaBox * ParaBoxPsrc=nullptr;
//    ParaBox * ParaBoxPdst=nullptr;
    ConfigBox * cConfigBox=nullptr;
    std::vector<ParaBox *> filter;
    bool filterFlag;
    while(ParaBoxPsrc == nullptr or ParaBoxPdst == nullptr){
        filterFlag=false;
        cParaBox=_Q.top();
        _Q.pop();
        if(ParaBoxPsrc == nullptr){
            if(cParaBox->isIn(_Psrc)){
                ParaBoxPsrc=cParaBox;
                filterFlag=true;
            }
        }
        if(ParaBoxPdst == nullptr){
            if(cParaBox->isIn(_Pdst)){
                ParaBoxPdst=cParaBox;
                filterFlag=true;
            }
        }
        if(filterFlag==false) filter.push_back(cParaBox);
    }
    //将所有不交盒子装回_Q
    for(auto it=filter.begin();it!=filter.end();it++) _Q.push(*it);
    
    //将 ParaBoxPsrc 与 ParaBoxPdst 细化为两个BFREE盒子
    bool Psrc_Flag=false;
    bool Pdst_Flag=false;
    ParaBox * nxtParaBoxPsrc=nullptr;
    ParaBox * nxtParaBoxPdst=nullptr;
    UnionFind UF;
    while(Psrc_Flag!=true or Pdst_Flag!=true){
        //如果盒子相同
        if(ParaBoxPsrc==ParaBoxPdst){
            cConfigBox= new ConfigBox(ParaBoxPsrc);
            if(cConfigBox->predicate()==BFREE){
                Psrc_Flag=true;
                UF.Push(ParaBoxPsrc);
                nxtParaBoxPsrc=ParaBoxPsrc;
                Pdst_Flag=true;
                UF.Push(ParaBoxPdst);
                nxtParaBoxPdst=ParaBoxPdst;
                std::cout<<"|  [BFREE] (Psrc & Pdst) in "<<*ParaBoxPsrc;
            }
            else{
                std::cout<<"|  [BMIXED] (Psrc & Pdst) in "<<*ParaBoxPsrc;
                assert(ParaBoxPsrc->SPLIT(_epsTrans, _epsRot));
                for(auto it=ParaBoxPsrc->_childrens.begin();it!=ParaBoxPsrc->_childrens.end();it++){
                    if((*it)->isIn(_Psrc)) nxtParaBoxPsrc=*it;
                    if((*it)->isIn(_Pdst)) nxtParaBoxPdst=*it;
                }
                for(auto it=ParaBoxPsrc->_childrens.begin();it!=ParaBoxPsrc->_childrens.end();it++){
                    if((*it)!=nxtParaBoxPsrc and (*it)!=nxtParaBoxPdst){
                        _Q.push(*it);
                    }
                }
                delete cConfigBox;
            }
        }
        //如果盒子不同
        else{
            if(Psrc_Flag==false){//还未找到包含Psrc的BFREE盒子
                cConfigBox= new ConfigBox(ParaBoxPsrc);
                if(cConfigBox->predicate()==BFREE){
                    Psrc_Flag=true;
                    UF.Push(ParaBoxPsrc);
                    nxtParaBoxPsrc=ParaBoxPsrc;
                    std::cout<<"|  [BFREE] Psrc in "<<*ParaBoxPsrc;
                }
                else{
                    std::cout<<"|  [BMIXED] Psrc in "<<*ParaBoxPsrc;
                    assert(ParaBoxPsrc->SPLIT(_epsTrans, _epsRot));
                    for(auto it=ParaBoxPsrc->_childrens.begin();it!=ParaBoxPsrc->_childrens.end();it++){
                        if((*it)->isIn(_Psrc)) {
                            nxtParaBoxPsrc=*it;
                            break;
                        }
                    }
                    for(auto it=ParaBoxPsrc->_childrens.begin();it!=ParaBoxPsrc->_childrens.end();it++){
                        if((*it)!=nxtParaBoxPsrc) _Q.push(*it);
                    }
                }
                delete cConfigBox;
            }
            if(Pdst_Flag==false){//还未找到包含Pdst的BFREE盒子
                cConfigBox= new ConfigBox(ParaBoxPdst);
                if(cConfigBox->predicate()==BFREE){
                    Pdst_Flag=true;
                    UF.Push(ParaBoxPdst);
                    nxtParaBoxPdst=ParaBoxPdst;
                    std::cout<<"|  [BFREE] Pdst in "<<*ParaBoxPdst;
                }
                else{
                    std::cout<<"|  [BMIXED] Pdst in "<<*ParaBoxPdst;
                    assert(ParaBoxPdst->SPLIT(_epsTrans, _epsRot));
                    for(auto it=ParaBoxPdst->_childrens.begin();it!=ParaBoxPdst->_childrens.end();it++){
                        if((*it)->isIn(_Pdst)){
                            nxtParaBoxPdst=*it;
                            break;
                        }
                    }
                    for(auto it=ParaBoxPdst->_childrens.begin();it!=ParaBoxPdst->_childrens.end();it++){
                        if((*it)!=nxtParaBoxPdst) _Q.push(*it);
                    }
                }
                delete cConfigBox;
            }
        }
        ParaBoxPsrc=nxtParaBoxPsrc;
        ParaBoxPdst=nxtParaBoxPdst;
        std::cout<<"|"<<std::endl;
    }
    
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] Finding 2 BFREE ParaBox"<<std::endl;
    std::cout<<"|  [UnionFind log] ";
    UF.PrintUnionFindLog();
    std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
    std::cout<<"|"<<std::endl;
    
    std::cout<<"|-3--------"<<std::endl<<"|"<<std::endl;
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] Searching ..."<<std::endl;
    int keepdepth=_Q.top()->_depth;
    int cloc=0;
    int BStuckSize=0;
    while(UF.isConnect(ParaBoxPsrc, ParaBoxPdst)==false){
//        if(_Q.size()==0){
//            std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] NO PATH FOUND!"<<std::endl;
//            return false;
//        }
        
        if((++cloc)%8000 == 0){
            std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] "<<cloc<<" box tested"<<std::endl;
            if(cloc%80000 == 0){
                std::cout<<"|"<<std::endl<<"|  [UnionFind log] ";
                UF.PrintUnionFindLog();
                std::cout<<"|  [BStuck log] BSTUCK.size() = "<<BStuckSize<<std::endl;
                std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
                std::cout<<"|"<<std::endl;
            }
        }
        //当堆提取的最小深度变化了,不再需要保存之前的Rod_1 Rod_2 Trans资源了
        if(keepdepth!=_Q.top()->_depth){
            std::cout<<"|"<<std::endl;
            std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] All box of depth ["<<keepdepth<<"] has already been tested"<<std::endl;
            std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] clear depth ["<<keepdepth<<"] cache"<<std::endl;
            std::cout<<"|  [(prev)Repo log] Rod_1 = "<<ConfigBox::Rod_1Repo.size()<<" ; Rod_2Repo.size() = "<<ConfigBox::Rod_2Repo.size()<<" ; TransObstacle = "<<ConfigBox::TransRepo.size()<<std::endl;
            clearRod_1Repo();
            clearRod_2Repo();
            clearTransRepo();
            std::cout<<"|  [(after)Repo log] Rod_1 = "<<ConfigBox::Rod_1Repo.size()<<" ; Rod_2Repo.size() = "<<ConfigBox::Rod_2Repo.size()<<" ; TransObstacle = "<<ConfigBox::TransRepo.size()<<std::endl;
            keepdepth = _Q.top()->_depth;//开始记录 下一层depth
//            std::cout<<"|  [UnionFind log] ";
//            UF.PrintUnionFindLog();
//            std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
            std::cout<<"|"<<std::endl;
        }
        
        cParaBox=_Q.top();
        _Q.pop();

        cConfigBox= new ConfigBox(cParaBox);
        cConfigBox->predicate();
        delete cConfigBox;
        if(cParaBox->_ConfigBoxStatus==BFREE){
            UF.Push(cParaBox);
            assert(cParaBox->_ConfigBoxStatus==BFREE);
            for(auto it=cParaBox->_neighbors.begin();it!=cParaBox->_neighbors.end();it++){
                if((*it)->_ConfigBoxStatus==BFREE){
                    UF.Union((*it), cParaBox);
                }
            }
        }
        else if(cParaBox->_ConfigBoxStatus==BMIXED){
            if(cParaBox->SPLIT(_epsTrans,_epsRot)==false){
                std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] NO-PATH with epsTrans = "<<_epsTrans<<" ; expRot = "<<_epsRot<<std::endl;
                return false;
            }
            for(auto it=cParaBox->_childrens.begin();it!=cParaBox->_childrens.end();it++){
                _Q.push(*it);
            }
        }
        else{
            assert(cParaBox->_ConfigBoxStatus==BSTUCK);
            BStuckSize++;
        }
    }
    std::cout<<"|"<<std::endl;
    std::cout<<"|----------"<<std::endl<<"|"<<std::endl;
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] Find a path "<<std::endl;
    std::cout<<"| *["<<(clock()-time_start)/(double)CLOCKS_PER_SEC<<" s] "<<cloc<<" box tested"<<std::endl;
    std::cout<<"|  [UnionFind log] ";
    UF.PrintUnionFindLog();
    std::cout<<"|  [BStuck log] BSTUCK.size() = "<<BStuckSize<<std::endl;
    std::cout<<"|  [_Q log] _Q.size() = "<<_Q.size()<<" ; _Q_min_depth = "<<_Q.top()->_depth<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"|----------"<<std::endl;

    
    return true;
}


std::vector<ParaBox *> SSS::_RoadBoxes;
std::vector<std::vector<double>> SSS::_RoadPoints;

bool SSS::BuildRoadMap(){
    std::unordered_map<ParaBox *,ParaBox *> BackPropMap;
    std::queue<ParaBox *> BFS;
    BackPropMap[ParaBoxPsrc]=nullptr;
    BFS.push(ParaBoxPsrc);
    ParaBox * cParaBox;
    
    while(BackPropMap.count(ParaBoxPdst)==0 and BFS.empty()==0){
        cParaBox=BFS.front();
        BFS.pop();
        for(auto it=cParaBox->_neighbors.begin();it!=cParaBox->_neighbors.end();it++){
            if((*it)->_ConfigBoxStatus==BFREE and BackPropMap.count(*it)==0){
                BackPropMap[*it]=cParaBox;
                BFS.push(*it);
            }
        }
    }
    assert(BFS.empty()==0);
    
    
    cParaBox = ParaBoxPdst;
    std::stack<ParaBox *> Stack;
    while(cParaBox!=nullptr){
        Stack.push(cParaBox);
        cParaBox=BackPropMap[cParaBox];
    }
    
    std::cout<<std::endl<<"|---------- <<PATH Log>> ----------"<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"| PathPlanning Mode: ["<<(ConfigBox::_PathPlanningMode==Insitu?"Insitu]":"Movable]")<<std::endl;
    std::cout<<"| Enable BSTUCK test ? : ["<<(IncludeBSTUCKtest==true?"True]":"False]")<<std::endl;
    std::cout<<"|"<<std::endl;
    std::cout<<"| FROM ("<<_Psrc[0]<<"_x, \t"<<_Psrc[1]<<"_y, \t"<<_Psrc[2]<<"_rod1, \t"<<_Psrc[3]<<"_rod2)"<<std::endl;
    std::cout<<"| TO   ("<<_Pdst[0]<<"_x, \t"<<_Pdst[1]<<"_y, \t"<<_Pdst[2]<<"_rod1, \t"<<_Pdst[3]<<"_rod2)"<<std::endl;
    std::cout<<"|"<<std::endl;
    
    _RoadBoxes.clear();
    _RoadPoints.clear();
    
    _RoadPoints.push_back(_Psrc);
    ParaBox * preParaBox=ParaBoxPsrc;
    int step=0;
    while(Stack.empty()==0){
        if(step!=0){
            _RoadPoints.push_back(FindRoadPoint(preParaBox, Stack.top()));
            preParaBox=Stack.top();
        }
        std::cout<<"|    ("<<_RoadPoints.back()[0]<<"_x, \t"<<_RoadPoints.back()[1]<<"_y, \t"<<_RoadPoints.back()[2]<<"_rod1, \t"<<_RoadPoints.back()[3]<<"_rod2)"<<std::endl;
        std::cout<<"|  /"<<std::endl;;
        std::cout<<"| ["<<step++<<"]\t"<<*(Stack.top());
        std::cout<<"|  \\"<<std::endl;
        _RoadBoxes.push_back(Stack.top());
        Stack.pop();
    }
    _RoadPoints.push_back(_Pdst);
    std::cout<<"|    ("<<_RoadPoints.back()[0]<<"_x, \t"<<_RoadPoints.back()[1]<<"_y, \t"<<_RoadPoints.back()[2]<<"_rod1, \t"<<_RoadPoints.back()[3]<<"_rod2)"<<std::endl;
    
    std::cout<<"|----------"<<std::endl;
    return true;
    
}
