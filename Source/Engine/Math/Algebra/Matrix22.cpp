#include "Matrix22.hpp"
#include "Vector2.hpp"
#include <cmath>

namespace Engine2D
{
    // ========================================
    // 멤버 함수 구현
    // ========================================

    void Matrix22::SetRows(const Vector2& row1, const Vector2& row2)
    {
        xx = row1.x;
        xy = row1.y;
        yx = row2.x;
        yy = row2.y;
    }

    void Matrix22::SetColumns(const Vector2& col1, const Vector2& col2)
    {
        xx = col1.x;
        xy = col2.x;
        yx = col1.y;
        yy = col2.y;
    }

    void Matrix22::GetRows(Vector2& row1, Vector2& row2) const
    {
        row1.x = xx;
        row1.y = xy;
        row2.x = yx;
        row2.y = yy;
    }

    void Matrix22::GetColumns(Vector2& col1, Vector2& col2) const
    {
        col1.x = xx;
        col1.y = yx;
        col2.x = xy;
        col2.y = yy;
    }

    Vector2 Matrix22::GetRow(size_t i) const
    {
        if (i == 0)
            return Vector2(xx, xy);
        else
            return Vector2(yx, yy);
    }

    Vector2 Matrix22::GetColumn(size_t i) const
    {
        if (i == 0)
            return Vector2(xx, yx);
        else
            return Vector2(xy, yy);
    }

    // ========================================
    // 전역 연산자 함수들
    // ========================================

    Matrix22 operator+(const Matrix22& a, const Matrix22& b)
    {
        return Matrix22(
                        a.xx + b.xx, a.xy + b.xy,
                        a.yx + b.yx, a.yy + b.yy);
    }

    Matrix22 operator-(const Matrix22& a, const Matrix22& b)
    {
        return Matrix22(
                        a.xx - b.xx, a.xy - b.xy,
                        a.yx - b.yx, a.yy - b.yy);
    }

    Matrix22 operator*(const Matrix22& a, const Matrix22& matrix)
    {
        Matrix22 result;
        result.xx = a.xx * matrix.xx + a.xy * matrix.yx;
        result.xy = a.xx * matrix.xy + a.xy * matrix.yy;
        result.yx = a.yx * matrix.xx + a.yy * matrix.yx;
        result.yy = a.yx * matrix.xy + a.yy * matrix.yy;
        return result;
    }

    Matrix22 operator*(const Matrix22& matrix, Real real)
    {
        Matrix22 result;
        result.xx = matrix.xx * real;
        result.xy = matrix.xy * real;
        result.yx = matrix.yx * real;
        result.yy = matrix.yy * real;
        return result;
    }

    Matrix22 operator*(Real real, const Matrix22& matrix)
    {
        Matrix22 result;
        result.xx = matrix.xx * real;
        result.xy = matrix.xy * real;
        result.yx = matrix.yx * real;
        result.yy = matrix.yy * real;
        return result;
    }

    // ========================================
    // 유틸리티 함수들
    // ========================================

    // hadamard product - multiply its component
    Matrix22 HadamardProduct(const Matrix22& a, const Matrix22& b)
    {
        return Matrix22(
                        a.xx * b.xx, a.xy * b.xy,
                        a.yx * b.yx, a.yy * b.yy);
    }
}
