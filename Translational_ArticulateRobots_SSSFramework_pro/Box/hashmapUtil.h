//
//  hashmapUtil.h
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/6.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef hashmapUtil_h
#define hashmapUtil_h
#include <functional>
#include "ParaBox.hpp"
struct TransHashFunc{
    std::size_t operator()(const ParaBox &pb) const{
        return std::hash<double>()(pb.fdx_getL()) ^ (std::hash<double>()(pb.fdx_getR())>>1)^ (std::hash<double>()(pb.fdy_getL())<<1) ^ (std::hash<double>()(pb.fdy_getR()));
    }
};
struct TransEqual{
    bool operator () (const ParaBox &pb1, const ParaBox &pb2 ) const{
        return pb1._Ix==pb2._Ix and pb1._Iy==pb2._Iy;
    }
};


struct Rot_1HashFunc{
    std::size_t operator()(const ParaBox &pb) const{
        return std::hash<double>()(pb.fd1_getL()) ^ (std::hash<double>()(pb.fd1_getR())>>1);
    }
};
struct Rod_1Equal{
    bool operator () (const ParaBox &pb1, const ParaBox &pb2 ) const{
        return pb1._I1==pb2._I1;
    }
};

struct Rot_2HashFunc{
    std::size_t operator()(const ParaBox &pb) const{
        return std::hash<double>()(pb.fd1_getL()) ^ (std::hash<double>()(pb.fd1_getR())>>1)^ (std::hash<double>()(pb.fd2_getL())<<1) ^ (std::hash<double>()(pb.fd2_getR()));
    }
};
struct Rod_2Equal{
    bool operator () (const ParaBox &pb1, const ParaBox &pb2 ) const{
        return pb1._I1==pb2._I1 and pb1._I2==pb2._I2;
    }
};


#endif /* hashmapUtil_h */
