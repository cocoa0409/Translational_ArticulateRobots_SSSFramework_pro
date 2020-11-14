//
//  Vector_3.hpp
//  Translational_ArticulateRobots_SSSFramework_pro
//
//  Created by 兆吉 王 on 2020/5/5.
//  Copyright © 2020 兆吉 王. All rights reserved.
//

#ifndef Vector_3_hpp
#define Vector_3_hpp
#include "CONFIG.h"

class Vector_3
{
private:
    // 三维向量坐标
    double _x, _y, _z;
public:
    Vector_3(){}
    ~Vector_3(){}
    // 创建一个三维向量向量
    Vector_3 (double x, double y, double z):_x(x),_y(y),_z(z) { }
    Vector_3 (const Vector_3 & v){_x=v.x();_y=v.y();_z=v.z();}
    //赋值
    Vector_3& operator = (const Vector_3& v){
        _x=v.x();_y=v.y();_z=v.z();
        return *this;
    }

    // 设置三维向量三个方向上的坐标
    void set (double x, double y, double z) { _x=x; _y=y; _z=z; }
    // 三维向量归一化
//    Vector_3    normalize() const { return((*this) / norm()); }
    void normalize() {
        double nrm=norm();
        _x/=nrm;_y/=nrm;_z/=nrm;
    }
    double norm        () const { return sqrt(normSquared()); }
    double      normSquared () const { return _x*_x+_y*_y+_z*_z; }

    bool iszero()const {return normSquared()<RoundErrorbound;}
    
    // BOOL型操作运算符
    bool operator == (const Vector_3& v) const { return _x==v._x && _y==v._y && _z==v._z; }
    bool operator != (const Vector_3& v) const { return _x!=v._x || _y!=v._y || _z!=v._z; }

    // 常见的运算符
    Vector_3  operator +  (const Vector_3 &v) const { return Vector_3(_x+v._x, _y+v._y, _z+v._z); }
    Vector_3& operator += (const Vector_3 &v)       { _x+=v._x; _y+=v._y; _z+=v._z; return *this; }
    Vector_3  operator -  () const                 { return Vector_3(-_x, -_y, -_z); }
    Vector_3  operator -  (const Vector_3 &v) const { return Vector_3(_x-v._x, _y-v._y, _z-v._z); }
    Vector_3& operator -= (const Vector_3 &v)       { _x-=v._x; _y-=v._y; _z-=v._z; return *this; }
    Vector_3  operator *  (double s) const              { return Vector_3(_x*s, _y*s, _z*s); }
    Vector_3& operator *= (double s)                { _x*=s; _y*=s; _z*=s; return *this; }
    Vector_3  operator /  (double s) const          { assert(s); return (*this)* (1/s); }
    Vector_3& operator /= (double s)                { assert(s); return (*this)*=(1/s); }
    double x() const{return _x;}
    double y() const{return _y;}
    double z() const{return _z;}
    friend std::ostream & operator<<(std::ostream &out, Vector_3 & v);
};
 
 
//三维向量点积
inline double Dot (const Vector_3& l, const Vector_3& r)
{
  return l.x()*r.x() + l.y()*r.y() + l.z()*r.z();
}
 
// 三维向量叉积
inline Vector_3 Cross (const Vector_3& l, const Vector_3& r)
{
  return Vector_3(
    l.y()*r.z() - l.z()*r.y(),
    l.z()*r.x() - l.x()*r.z(),
    l.x()*r.y() - l.y()*r.x() );
}
 
// 三维向量混合积
double BlendProduct (const Vector_3& l, const Vector_3& m, const Vector_3& r);

// 判断平面与点集的关系
PointSetLocation WhichSide(const std::vector<Point_3> & S, const Vector_3 & D, const Point_3 & P);

#endif /* Vector_3_hpp */
