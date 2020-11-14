//
//  UnionFind.hpp
//  mech-arm-2dof-3d
//
//  Created by 兆吉 王 on 2019/8/21.
//  Copyright © 2019 兆吉 王. All rights reserved.
//

#ifndef UnionFind_hpp
#define UnionFind_hpp

#include "../Box/ParaBox.hpp"

class dSet {
public:
    ParaBox* pParaBox;
    dSet* pParent;  // parent in union find
    int rank;
    
    explicit dSet(ParaBox* b):pParaBox(b), rank(1) {
        pParent = this;
        b->pSet = this;
    }
};

class UnionFind {
private:
    dSet* pathCompress(dSet* set) {
        if (set->pParent == set) {
            return set;
        }
        
        set->pParent = pathCompress(set->pParent);
        return set->pParent;
    }
    
public:
    ParaBox* Find(ParaBox* b) {
        dSet* root = pathCompress(b->pSet);
        return root->pParaBox;
    }
    
    void Push(ParaBox* b){
        if(b->pSet == 0){
            new dSet(b);
            size_count_++;
            Component_count_ ++;
        }
    }
    
    bool isConnect(ParaBox* a, ParaBox* b) {
        if (Find(a) == Find(b)) {
            return true;
        }
        return false;
    }
    
    void Union(ParaBox*a, ParaBox* b) {
        dSet* roota = Find(a)->pSet;
        dSet* rootb = Find(b)->pSet;
        if (roota == rootb) {
            return;
        }
        if (roota->rank > rootb->rank) {
            rootb->pParent = roota;
        } else if (roota->rank == rootb->rank) {
            rootb->pParent = roota;
            ++roota->rank;
        } else {
            roota->pParent = rootb;
        }
        Component_count_ --;
    }
    void PrintUnionFindLog(){
        std::cout<<size_count_<<" boxes / "<<Component_count_<<" components"<<std::endl;
    }
    
    static int size_count_;
    static int Component_count_;
};


#endif /* UnionFind_hpp */
