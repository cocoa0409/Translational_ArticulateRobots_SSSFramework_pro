//
//  ParaBox.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include "ParaBox.hpp"

size_t ParaBox::_id_count=0;
size_t ParaBox::_obs_size=0;

ParaBox::ParaBox(ParaBox * parent,const IntervalT<double> & Ix,const IntervalT<double> & Iy,const IntervalT<double> & I1,const IntervalT<double> & I2){
    _parent=parent;
    _id=_id_count;
    _id_count++;
    _Ix=Ix;
    _Iy=Iy;
    _I1=I1;
    _I2=I2;
    if( fd2_width() <= 90){
        double mid = fd2_mid();
        if(mid <=0){
            if(mid <=-90) _FD2Status = N180toN90;
            else _FD2Status = N90toP0;
        }
        else{
            if(mid<=90) _FD2Status = P0toP90;
            else _FD2Status = P90toP180;
        }
    }
    else
        _FD2Status = INI;
    
    //从parent提取需要检测Rod_1/Rod_2子区间相交的障碍物信息
    if(_parent != nullptr){
        _ObssJudgeFlag_Rod_1.reserve(_obs_size);
        for(auto it=_parent->_ObssJudgeFlag_Rod_1.begin();it!=_parent->_ObssJudgeFlag_Rod_1.end();it++){
            _ObssJudgeFlag_Rod_1.push_back(*it);
        }
        _ObssJudgeFlag_Rod_2.reserve(_obs_size);
        for(auto it=_parent->_ObssJudgeFlag_Rod_2.begin();it!=_parent->_ObssJudgeFlag_Rod_2.end();it++){
            _ObssJudgeFlag_Rod_2.push_back(*it);
        }
    }
    //如果P->_parent不存在,或者P->,则全true,代表均需要检验;
    else{
        for(int i=0;i<_obs_size;i++){
            _ObssJudgeFlag_Rod_1.push_back(true);//true代表需要检查_Obss[i]与Rod_1的碰撞
            _ObssJudgeFlag_Rod_2.push_back(true);//true代表需要检查_Obss[i]与Rod_2的碰撞
        }
    }
    assert(_ObssJudgeFlag_Rod_1.size()==_obs_size);
    assert(_ObssJudgeFlag_Rod_2.size()==_obs_size);
}

bool ParaBox::isNeighbor(ParaBox * P){
    int fdxOverlap=Overlap(_Ix,P->_Ix)?1:0;
    int fdyOverlap=Overlap(_Iy,P->_Iy)?1:0;
    int fd1Overlap=Overlap(_I1,P->_I1)?1:0;
    int fd2Overlap=Overlap(_I2,P->_I2)?1:0;
    if(fdxOverlap+fdyOverlap+fd1Overlap+fd2Overlap<3) return false;
    
    if(fd2Overlap==0){
        if(fd2_getL()==P->fd2_getR() or fd2_getR()==P->fd2_getL() )
            return true;
    }
    if(fd1Overlap==0){
        if(fd1_getL()==P->fd1_getR() or fd1_getR()==P->fd1_getL() )
            return true;
    }
    if(fdxOverlap==0){
        if(fdx_getL()==P->fdx_getR() or fdx_getR()==P->fdx_getL() )
            return true;
    }
    if(fdyOverlap==0){
        if(fdy_getL()==P->fdy_getR() or fdy_getR()==P->fdy_getL() )
            return true;
    }
    return false;
}

bool ParaBox::isLeaf() {
    return _childrens.empty();
}

bool ParaBox::SPLIT(double eps_trans,double eps_rot){
    SPLITMODE splitmode;
    if(fdx_width()<eps_trans or fdy_width()<eps_trans){
        if(fd1_width()<eps_rot or fd2_width()<eps_rot)
            splitmode = NOOP;
        else
            splitmode = Rotonly;
    }
    else{
        if(fd1_width()<eps_rot or fd2_width()<eps_rot)
            splitmode = Transonly;
        else
            splitmode = TransRot;
    }
    
    double fd1l=fd1_getL(),fd1r=fd1_getR(),fd2l=fd2_getL(),fd2r=fd2_getR();
    double fd1mid = fd1_mid(),fd2mid = fd2_mid();
    
    double fdxl=fdx_getL(),fdxr=fdx_getR(),fdyl=fdy_getL(),fdyr=fdy_getR();
    double fdxmid = fdx_mid(),fdymid = fdy_mid();
    
    if(splitmode == NOOP)   return false;
    else if(splitmode == Transonly){
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdyl,fdymid},{fd1l,fd1r},{fd2l,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdyl,fdymid},{fd1l,fd1r},{fd2l,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdymid,fdyr},{fd1l,fd1r},{fd2l,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdymid,fdyr},{fd1l,fd1r},{fd2l,fd2r}));
    }
    else if(splitmode == Rotonly){
        _childrens.push_back(new ParaBox(this,{fdxl,fdxr},{fdyl,fdyr},{fd1l,fd1mid},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxr},{fdyl,fdyr},{fd1mid,fd1r},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxr},{fdyl,fdyr},{fd1l,fd1mid},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxr},{fdyl,fdyr},{fd1mid,fd1r},{fd2mid,fd2r}));
    }
    else if(splitmode == TransRot){
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdyl,fdymid},{fd1l,fd1mid},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdyl,fdymid},{fd1l,fd1mid},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdymid,fdyr},{fd1l,fd1mid},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdymid,fdyr},{fd1l,fd1mid},{fd2l,fd2mid}));
        
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdyl,fdymid},{fd1mid,fd1r},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdyl,fdymid},{fd1mid,fd1r},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdymid,fdyr},{fd1mid,fd1r},{fd2l,fd2mid}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdymid,fdyr},{fd1mid,fd1r},{fd2l,fd2mid}));
        
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdyl,fdymid},{fd1l,fd1mid},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdyl,fdymid},{fd1l,fd1mid},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdymid,fdyr},{fd1l,fd1mid},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdymid,fdyr},{fd1l,fd1mid},{fd2mid,fd2r}));
        
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdyl,fdymid},{fd1mid,fd1r},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdyl,fdymid},{fd1mid,fd1r},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxl,fdxmid},{fdymid,fdyr},{fd1mid,fd1r},{fd2mid,fd2r}));
        _childrens.push_back(new ParaBox(this,{fdxmid,fdxr},{fdymid,fdyr},{fd1mid,fd1r},{fd2mid,fd2r}));
    }
    else{   return false;}
    
    //针对每个children分别赋值与children中的neighbor
    for(auto it=_childrens.begin();it!=_childrens.end();it++){
        (*it)->_depth= _depth+1;
        for(auto mt=it+1;mt!=_childrens.end();mt++){
            if((*it)->isNeighbor(*mt)){
                (*it)->_neighbors.push_back(*mt);
                (*mt)->_neighbors.push_back(*it);
            }
        }
    }
    //针对每个parent的neighbor(neighbor包含非叶与叶节点,只考虑叶节点即可)
    //分别考虑children是否为neighbor
    for(auto it=_neighbors.begin();it!=_neighbors.end();it++){
        if((*it)->isLeaf()){
            for(auto mt=_childrens.begin();mt!=_childrens.end();mt++){
                if((*it)->isNeighbor(*mt)){
                    (*it)->_neighbors.push_back(*mt);
                    (*mt)->_neighbors.push_back(*it);
                }
            }
        }
    }
    return true;
}

bool ParaBox::isIn(std::vector<double> & v){
    assert(_FD2Status!= INI);
    if(v[0]>=fdx_getL() and v[0]<=fdx_getR() and v[1]>=fdy_getL() and v[1]<=fdy_getR() and v[2]>=fd1_getL() and v[2]<=fd1_getR() and v[3]>=fd2_getL() and v[3]<=fd2_getR())
        return true;
    else
        return false;
}




std::ostream & operator<<(std::ostream &out, ParaBox & P){
    out <<"id["<<P._id<<"] depth["<<P._depth<<"]\t"<<P._Ix <<"x  \t"<< P._Iy<<"y  \t"<< P._I1 <<"1  \t"<< P._I2<<"2"<<std::endl;
    return out;
}
    
    
std::vector<double> FindRoadPoint(ParaBox * a,ParaBox * b){
    assert(a->isNeighbor(b));
    std::vector<double> rp;
    rp.push_back(Intersect(a->_Ix, b->_Ix).mid());
    rp.push_back(Intersect(a->_Iy, b->_Iy).mid());
    rp.push_back(Intersect(a->_I1, b->_I1).mid());
    rp.push_back(Intersect(a->_I2, b->_I2).mid());
    
    return rp;
}
