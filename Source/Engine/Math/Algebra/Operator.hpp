#pragma once

#include "Vector2.hpp"
#include "Matrix22.hpp"
#include "../Utility/MathUtility.hpp"

namespace Engine2D
{
    // ========================================
    // 유틸리티 함수들
    // ========================================

    // generate tensor from two vector product
    Matrix22 OuterProduct(const Vector2& a, const Vector2& b);

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Vector2 operator*(const Matrix22& matrix, const Vector2& vector);
    Vector2 operator*(const Vector2& vector, const Matrix22& matrix);

    // ========================================
    // 수학적 연산 함수들
    // ========================================

    Vector2 SolveMatrix22(const Matrix22& a, const Vector2& b);

    class Vector2Pair
    {
    public:
        explicit Vector2Pair(const Vector2& a = Vector2(), const Vector2& b = Vector2())
            : a(a), b(b)
        {
        }

        explicit Vector2Pair(const Vector2Pair& rhs)
            : a(rhs.a), b(rhs.b)
        {
        }

        Vector2Pair& operator=(const Vector2Pair& rhs)
        {
            if (this != &rhs)
            {
                a = rhs.a;
                b = rhs.b;
            }
            return *this;
        }

    public:
        Vector2 a;
        Vector2 b;
    };

    Vector2 GetNormal(const Vector2& vector);
}
