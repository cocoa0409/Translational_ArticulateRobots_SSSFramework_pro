//
//  IntervalT.h
//  mech-arm-2dof-3d
//
//  Created by 兆吉 王 on 2019/6/4.
//  Copyright © 2019 兆吉 王. All rights reserved.
//

#ifndef IntervalT_h
#define IntervalT_h
#include <cmath>
enum SignCondition { _POS, _NEG, _ZERO, _NONNEG, _NONPOS, _NONZERO, _UNKNOWN };
#include <iostream>
#include <iomanip>
template <typename NT> class IntervalT {
public:
    // Construct an interval from its left and right extents.
    IntervalT(const NT &a, const NT &b) : left_(a), right_(b) { }
    // This is ineffective but turns out to be reqd. Many templated
    // algorithms require the creation of naked types.
    IntervalT() : left_(0), right_(0) { }
    // Constructors for constructing thin intervals from the underlying
    // number types and a special case of integers for the = 0 case.
    IntervalT(const int a) : left_(a), right_(a) { }
    IntervalT(const NT &a) : left_(a), right_(a) { }
    // Copy constructor : Open question : Do we allow this ?
    //
    // The suppression of this constructor forces this class
    // to be passed around / returned by reference / pointer
    // only. This in general leads to better performance at
    // Level 2 and Level 3 where BigFloat and Expr are large
    // classes. But the downside is that an Interval will always
    // be returned by pointer, and will therefore be heap allocated
    // and will need to be freed.
    IntervalT(const IntervalT<NT> &i) {
        left_ = i.left_;
        right_ = i.right_;
    }
    
    // Get the "left" extent of this interval. Note that this is a
    // reference, assign it to a value type if you want to keep it
    // around after the lifetime of this class.
    const NT &getL() const {
        return left_;
    }
    // Get the "right" extent of this interval.
    const NT &getR() const {
        return right_;
    }
    // Return true iff. this interval contains a zero.
    // (Chee: it would be clearer to return a _ZERO or _NONZERO sign condition)
    const bool zero() const {
        return (left_ <= 0) && (right_ >= 0);
    }
    
    // This definition of SignCondition should be used more widely!
    // Return a sign condition (_ZERO, _POS, _NEG, _UNKNOWN):
    const SignCondition sign() const {
        if (left_ <= 0) {
            if (right_ >= 0) {
                return(_ZERO);
            } else {
                // This corresponds to left_ <= 0 && right_ <= 0.
                return(_NEG);
            }
        }
        
        // This is left_ > 0, which is positive.
        return(_POS);
    }
    // Return the mid point of this interval.
    const NT mid() const {
        // This will almost certainly be an error in this case.
        // There is no sensible midpoint if the left or right are
        // infinite.
        assert (left_ != I_NEG_INFTY && right_ != I_POS_INFTY);
        
        return (right_ + left_) / 2;
    }
    const NT width() const {
        // The same argument as above, if one is using the width
        // of an infinite interval for something its almost certainly
        // an abuse rather than a use.
        assert (left_ != I_NEG_INFTY && right_ != I_POS_INFTY);
        return right_ - left_;
    }
    
    const NT halfwidth() const {
        // The same argument as above, if one is using the width
        // of an infinite interval for something its almost certainly
        // an abuse rather than a use.
        assert (left_ != I_NEG_INFTY && right_ != I_POS_INFTY);
        return (right_ - left_) / 2;
    }
    
    // Equating an interval to a number, this should construct a thin
    // (degenerate interval with that number.
    IntervalT<NT> &operator=(const NT &r) {
        left_ = r;
        right_ = r;
        return *this;
    }
    // The same with an integer.
    IntervalT<NT> &operator=(const int r) {
        left_ = r;
        right_ = r;
        return *this;
    }
    
    // Standard operations on intervals. We currently define
    // the self modifying operators inline. The four standard
    // operations on intervals are (as with numbers) multiplication
    // addition, subraction and division.
    IntervalT<NT>& operator*=(const IntervalT<NT> &t) {
        NT temp_a = (std::min)(
                               (std::min)(left_ * t.left_, left_ * t.right_),
                               (std::min)(right_ * t.left_, right_ * t.right_));
        right_ = (std::max)(
                            (std::max)(left_ * t.left_, left_ * t.right_),
                            (std::max)(right_ * t.left_, right_ * t.right_));
        left_ = temp_a;
        
        return *this;
    }
    IntervalT<NT>& operator+=(const IntervalT<NT> &s) {
        left_ += s.left_;
        right_ += s.right_;
        
        return *this;
    }
    IntervalT<NT>& operator-=(const IntervalT<NT> &s) {
        left_ = left_ - s.right_;
        right_ = right_ - s.left_;
        
        return *this;
    }
    IntervalT<NT>& operator/=(const IntervalT<NT> &t) {
        // If you need division by intervals that contain a zero,
        // use the functions under the extended operators namespace.
        if (t.zero()) {
            assert (false);
            // Never reached.
            return (*this);
        }
        
        NT temp_a = (std::min)(
                               (std::min)(left_ /t.left_, left_/t.right_),
                               (std::min)(right_/t.left_, right_/t.right_));
        right_ = (std::max)(
                            (std::max)(left_ /t.left_, left_/t.right_),
                            (std::max)(right_/t.left_, right_/t.right_));
        left_ = temp_a;
        
        return *this;
    }
    
    // This Expand function is initially used in box function,
    // to expand, pass desired factor as function parameter
    // We might not want this function to be self-modified because
    // we need to keep the initial interval as well
    // Chee and Shang Wang (Aug 2011)
    IntervalT<NT> Expand(const NT factor) const {
        const NT &w = this->width();
        assert (left_ != I_NEG_INFTY && right_ != I_POS_INFTY);
        assert (factor >= 1);
        return IntervalT<NT>((left_-0.5*w*(factor-1)), (right_+0.5*w*(factor-1)));
    }
    
    // similar as above
    IntervalT<NT> Half() const {
        const NT &w = this->width();
        assert (left_ != I_NEG_INFTY && right_ != I_POS_INFTY);
        return IntervalT<NT>((left_+0.25*w), (right_-0.25*w));
    }
    
    
    // Tests for equality with other intervals, including
    // numbers (the thin case). For now these definitions
    // are not symmetric ie. the number types BigFloat etc.
    // have tests for equality with Intervals.
    bool operator==(const IntervalT<NT> &rhs) const {
        return (this == &rhs) || (left_ == rhs.left_ && right_ == rhs.right_);
    }
    bool operator!=(const IntervalT<NT> &rhs) const {
        return !((*this) == rhs);
    }
    bool operator==(const NT &rhs) const {
        return left_ == rhs && right_ == rhs;
    }
    bool operator!=(const NT &rhs) const {
        return !((*this) == rhs);
    }
    
    // Used for intervals of infinite width. This whole business of
    // representing infinity in this way is rather crude, and might
    // cause trouble. (CORE_posInfty = std::numeric_limits<unsigned long>::max).
    static const NT I_POS_INFTY;
    static const NT I_NEG_INFTY;
private:
    // The extents of this interval.
    NT left_;
    NT right_;
};


template <typename NT> const NT IntervalT<NT>::I_POS_INFTY = __DBL_MAX__;
template <typename NT> const NT IntervalT<NT>::I_NEG_INFTY = -__DBL_MAX__;

template <typename NT>
inline const bool weakOverlap(const IntervalT<NT> &s, const IntervalT<NT> &t) {
    return !(s.getR() < t.getL() || s.getL() > t.getR());
}

template <typename NT>
inline const bool Overlap(const IntervalT<NT> &s, const IntervalT<NT> &t) {
    return !(s.getR() <= t.getL() || s.getL() >= t.getR());
}
// Note that this function returns a sensible result only if
// these intervals actually overlap, so this should be called in the form
// if (overlap(a, b)) { intersect(a, b) }
template <typename NT>
inline IntervalT<NT> Intersect(const IntervalT<NT> &a, const IntervalT<NT> &b) {
    return IntervalT<NT>((std::max)(a.getL(), b.getL()), (std::min)(a.getR(), b.getR()));
}
// Note that a consequence of this [-INF, INF ] is contained in
// [-INF, INF ].
template <typename NT>
inline const bool Contains(const IntervalT<NT> &a, const IntervalT<NT> &b) {
    return (b.getL() >= a.getL()) && (b.getR() <= a.getR());
}

// For now implemented in terms of self modifying operators, we
// can make these faster if required, at the cost of some code duplication.
template <typename NT>
inline IntervalT<NT> operator-(const IntervalT<NT> &lhs, const IntervalT<NT> &rhs) {
    IntervalT<NT> ret = lhs;
    ret-=rhs;
    return ret;
}
template <typename NT>
inline IntervalT<NT> operator+(const IntervalT<NT> &lhs, const IntervalT<NT> &rhs) {
    IntervalT<NT> ret = lhs;
    ret+=rhs;
    return ret;
}
template <typename NT>
inline IntervalT<NT> operator*(const IntervalT<NT> &lhs, const IntervalT<NT> &rhs) {
    IntervalT<NT> ret = lhs;
    ret*=rhs;
    return ret;
}
template <typename NT>
inline IntervalT<NT> operator/(const IntervalT<NT> &lhs, const IntervalT<NT> &rhs) {
    IntervalT<NT> ret = lhs;
    ret/=rhs;
    return ret;
}
// Multiplication with a scalar.
// NOTE(narayan): This operator is potentially ambiguous.
template <typename NT>
inline IntervalT<NT> operator*(const IntervalT<NT> &lhs, const NT &rhs) {
    if (rhs < 0) {
        return IntervalT<NT>(lhs.getR() * rhs, lhs.getL() * rhs);
    }
    
    return IntervalT<NT>(lhs.getL() * rhs, lhs.getR() * rhs);
}

template <typename NT>
inline std::ostream& operator<<(std::ostream& o, const IntervalT<NT>& v) {
//    std::cout.setf(std::ios::fixed,std::ios::floatfield);
    o << "[ ";
    if (v.getL() == IntervalT<NT>::I_NEG_INFTY) {
        o << "-INF";
    } else {
        o << v.getL();
    }
    
    o << ", ";
    
    if (v.getR() == IntervalT<NT>::I_POS_INFTY) {
        o << "INF";
    } else {
        o << v.getR();
    }
    o << " ]";
    
    return o;
}

#endif /* IntervalT_h */
