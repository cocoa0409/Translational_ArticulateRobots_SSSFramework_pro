//
//  main.cpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#include <iostream>
#include <unordered_map>

class ParaBox{
public:
    double a;
    double b;
    double c;
    ParaBox(double ai,double bi, double ci):a(ai),b(bi),c(ci){};
};


#include <functional>
struct HashFunc
{
    std::size_t operator()(const ParaBox &pb) const
    {
        return std::hash<double>()(pb.a) ^ (std::hash<double>()(pb.b)>>1);
    }
};


struct EqualKey
{
    bool operator () (const ParaBox &pb1, const ParaBox &pb2 ) const
    {
        return pb1.a==pb2.a and pb1.b==pb2.b;
    }
};


int main(int argc, const char * argv[]) {
    std::unordered_map<ParaBox,int,HashFunc,EqualKey> umap;
    umap.insert({{999,666,4}, 1237125123});
    umap.insert({{666,999,4}, 19125123});
    std::cout<<umap.count({999,666,1});
}
