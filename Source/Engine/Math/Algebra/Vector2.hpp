#pragma once

#include "../Utility/MathConstants.hpp"
#include "../Utility/MathUtility.hpp"

namespace Engine2D
{
    class Vector2
    {
    public:
        Real x = 0, y = 0;

    public:
        Vector2()  = default;
        ~Vector2() = default;

        Vector2(const Vector2& rhs)
            : x(rhs.x), y(rhs.y)
        {
        }

        Vector2(Real x, Real y)
            : x(x), y(y)
        {
        }

        explicit Vector2(Real a[2])
            : x(a[0]), y(a[1])
        {
        }

        void Set(Real x, Real y)
        {
            this->x = x;
            this->y = y;
        }

        void SetZero()
        {
            x = 0.0;
            y = 0.0;
        }

        void SetInverse()
        {
            x = Math::IsZero(x) ? 0.0 : 1.0 / x;
            y = Math::IsZero(y) ? 0.0 : 1.0 / y;
        }

        void SetNegate()
        {
            x = -x;
            y = -y;
        }

        void SetNormalize()
        {
            Real length = Math::Sqrt(x * x + y * y);
            if (length > 0.0)
            {
                (*this) *= (1.0 / length);
            }
        }

        void ClearError()
        {
            x = ClearError(x);
            y = ClearError(y);
        }

        Real Length() const
        {
            return Math::Sqrt(x * x + y * y);
        }

        Real LengthSq() const
        {
            return x * x + y * y;
        }

        Real SmallestCompo() const
        {
            return x < y ? x : y;
        }

        Real LargestCompo() const
        {
            return x > y ? x : y;
        }

        Real    DistanceTo(const Vector2& rhs) const;
        Real    DistanceSqTo(const Vector2& rhs) const;
        Vector2 ProjectTo(const Vector2& rhs) const;
        Vector2 ProjectFrom(const Vector2& rhs) const;

        Vector2 Unit() const
        {
            Real length = Math::Sqrt(x * x + y * y);
            length      = length > 0.0 ? 1.0 / length : 0.0;
            return Vector2(x * length, y * length);
        }

        Vector2 Half() const
        {
            return Vector2(0.5 * x, 0.5 * y);
        }

        Vector2 Inverse() const
        {
            return Vector2(
                           Math::IsZero(x) ? 0.0 : 1.0 / x,
                           Math::IsZero(y) ? 0.0 : 1.0 / y);
        }

        Vector2 Negate() const
        {
            return Vector2(-x, -y);
        }

        Vector2 Scale(Real scale) const
        {
            return Vector2(scale * x, scale * y);
        }

        Vector2 Absolute() const
        {
            return Vector2(Math::Abs(x), Math::Abs(y));
        }

        bool IsValid() const
        {
            return Math::IsValid(x) && Math::IsValid(y);
        }

        bool IsZero() const
        {
            return Math::IsZero(x) && Math::IsZero(y);
        }

        Real operator[](size_t i) const
        {
            return (&x)[i];
        }

        Real& operator[](size_t i)
        {
            return (&x)[i];
        }

        Real operator()(size_t i) const
        {
            return (&x)[i];
        }

        Real& operator()(size_t i)
        {
            return (&x)[i];
        }

        Vector2& operator =(const Vector2& rhs)
        {
            if (this != &rhs)
            {
                x = rhs.x;
                y = rhs.y;
            }
            return *this;
        }

        Vector2& operator =(Real rhs)
        {
            x = rhs;
            y = rhs;
            return *this;
        }

        bool operator ==(const Vector2& rhs) const
        {
            return Math::IsEqual(x, rhs.x) && Math::IsEqual(y, rhs.y);
        }

        bool operator !=(const Vector2& rhs) const
        {
            return Math::IsNotEqual(x, rhs.x) || Math::IsNotEqual(y, rhs.y);
        }

        Vector2 operator -() const
        {
            return Vector2(-x, -y);
        }

        Vector2& operator +=(const Vector2& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        Vector2& operator -=(const Vector2& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        Vector2& operator +=(Real real)
        {
            x += real;
            y += real;
            return *this;
        }

        Vector2& operator -=(Real real)
        {
            x -= real;
            y -= real;
            return *this;
        }

        Vector2& operator *=(Real real)
        {
            x *= real;
            y *= real;
            return *this;
        }

        Vector2& operator /=(Real real)
        {
            x /= real;
            y /= real;
            return *this;
        }

    };

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Vector2 operator +(const Vector2& a, const Vector2& b);
    Vector2 operator -(const Vector2& a, const Vector2& b);
    Vector2 operator*(Real real, const Vector2& vector);
    Vector2 operator*(const Vector2& vector, Real real);
    Vector2 operator /(const Vector2& vector, Real real);

    // ========================================
    // 유틸리티 함수들
    // ========================================

    // dot product
    Real DotProduct(const Vector2& a, const Vector2& b);

    // cross product: (a.x, a.y, 0) X (b.x, b.y, 0) = (0, 0, result)
    Real CrossProduct(const Vector2& a, const Vector2& b);

    // cross product: (0, 0, s) X (v.x, v.y, 0) = (res.x, res.y, 0)
    Vector2 CrossProduct(Real s, const Vector2& v);

    // cross product: (v.x, v.y, 0) X (0, 0, s) = (res.x, res.y, 0)
    Vector2 CrossProduct(const Vector2& v, Real s);

    // hadamard product - multiply its component
    Vector2 HadamardProduct(const Vector2& a, const Vector2& b);

    Real Distance(const Vector2& a, const Vector2& b);
    Real DistanceSq(const Vector2& a, const Vector2& b);
    Vector2 Project(const Vector2& a, const Vector2& b);
}
