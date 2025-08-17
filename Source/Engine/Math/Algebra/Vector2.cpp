#include "Vector2.hpp"
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 멤버 함수 구현
    // ========================================

    Real Vector2::DistanceTo(const Vector2& rhs) const
    {
        Real x_term = rhs.x - x;
        Real y_term = rhs.y - y;
        return Math::Sqrt(x_term * x_term + y_term * y_term);
    }

    Real Vector2::DistanceSqTo(const Vector2& rhs) const
    {
        Real x_term = rhs.x - x;
        Real y_term = rhs.y - y;
        return x_term * x_term + y_term * y_term;
    }

    Vector2 Vector2::ProjectTo(const Vector2& rhs) const
    {
        return (DotProduct(*this, rhs) / DotProduct(rhs, rhs)) * rhs;
    }

    Vector2 Vector2::ProjectFrom(const Vector2& rhs) const
    {
        return DotProduct(rhs, *this) / DotProduct(*this, *this) * *this;
    }

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Vector2 operator +(const Vector2& a, const Vector2& b)
    {
        return Vector2(a.x + b.x, a.y + b.y);
    }

    Vector2 operator -(const Vector2& a, const Vector2& b)
    {
        return Vector2(a.x - b.x, a.y - b.y);
    }

    Vector2 operator*(Real real, const Vector2& vector)
    {
        return Vector2(vector.x * real, vector.y * real);
    }

    Vector2 operator*(const Vector2& vector, Real real)
    {
        return Vector2(vector.x * real, vector.y * real);
    }

    Vector2 operator /(const Vector2& vector, Real real)
    {
        return Vector2(vector.x / real, vector.y / real);
    }

    // ========================================
    // 유틸리티 함수들
    // ========================================

    // dot product
    Real DotProduct(const Vector2& a, const Vector2& b)
    {
        return (a.x * b.x + a.y * b.y);
    }

    // cross product: (a.x, a.y, 0) X (b.x, b.y, 0) = (0, 0, result)
    Real CrossProduct(const Vector2& a, const Vector2& b)
    {
        return (a.x * b.y - a.y * b.x);
    }

    // cross product: (0, 0, s) X (v.x, v.y, 0) = (res.x, res.y, 0)
    Vector2 CrossProduct(Real s, const Vector2& v)
    {
        return Vector2(-s * v.y, s * v.x);
    }

    // cross product: (v.x, v.y, 0) X (0, 0, s) = (res.x, res.y, 0)
    Vector2 CrossProduct(const Vector2& v, Real s)
    {
        return Vector2(s * v.y, -s * v.x);
    }

    // hadamard product - multiply its component
    Vector2 HadamardProduct(const Vector2& a, const Vector2& b)
    {
        return Vector2(a.x * b.x, a.y * b.y);
    }

    Real Distance(const Vector2& a, const Vector2& b)
    {
        Real x_term = b.x - a.x;
        Real y_term = b.y - a.y;
        return Math::Sqrt(x_term * x_term + y_term * y_term);
    }

    Real DistanceSq(const Vector2& a, const Vector2& b)
    {
        Real x_term = b.x - a.x;
        Real y_term = b.y - a.y;
        return x_term * x_term + y_term * y_term;
    }

    Vector2 Project(const Vector2& a, const Vector2& b)
    {
        return (DotProduct(a, b) / DotProduct(b, b)) * b;
    }
}
